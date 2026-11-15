"""Sessão segura com o Lite 6.

safe_session() liga ao controlador, limpa erros e avisos, ativa o movimento,
configura e liga o modo reduzido e confirma-o por leitura (velocidades e gamas
de junta) antes de devolver o braço. home() só move o braço se o modo reduzido
estiver ligado.

Todos os valores vêm de config/robot.yaml (graus, °/s, mm/s). Qualquer código
de retorno diferente de 0 levanta RuntimeError antes de qualquer movimento.
"""
from pathlib import Path

import yaml
from xarm.wrapper import XArmAPI

CONFIG = Path(__file__).resolve().parent.parent / 'config' / 'robot.yaml'
TOL = 0.01      # tolerância na leitura dos limites (°/s, mm/s, graus)


def load_config(path=CONFIG):
    with open(path) as f:
        return yaml.safe_load(f)


def _check(code, what):
    """Levanta RuntimeError se uma chamada do SDK não devolver 0."""
    if code != 0:
        raise RuntimeError(f'{what} falhou com código {code}')


def safe_session(config_path=CONFIG):
    """Liga ao Lite 6 em modo reduzido confirmado. Devolve o XArmAPI ligado."""
    cfg = load_config(config_path)
    red = cfg['reduced']
    check_limits = cfg.get('sdk_check_joint_limit', True)
    if not check_limits:
        print('AVISO: verificação local de gamas de junta do SDK desligada '
              '(usar só com o simulador de firmware)')
    arm = XArmAPI(cfg['ip'], is_radian=False, check_joint_limit=check_limits)
    try:
        if not arm.connected:
            raise RuntimeError(f'sem ligação ao controlador em {cfg["ip"]}')

        _check(arm.clean_error(), 'clean_error')
        _check(arm.clean_warn(), 'clean_warn')
        _check(arm.motion_enable(enable=True), 'motion_enable')
        _check(arm.set_mode(0), 'set_mode(0)')
        _check(arm.set_state(0), 'set_state(0)')

        # Os limites só têm efeito depois de (re)ligar o modo reduzido.
        _check(arm.set_reduced_max_joint_speed(red['joint_speed']),
               'set_reduced_max_joint_speed')
        _check(arm.set_reduced_max_tcp_speed(red['tcp_speed']),
               'set_reduced_max_tcp_speed')
        _check(arm.set_reduced_mode(True), 'set_reduced_mode(True)')

        # Confirmação por leitura: velocidades.
        code, states = arm.get_reduced_states()
        _check(code, 'get_reduced_states')
        on, _, tcp, joint = states[:4]
        if not (on and abs(tcp - red['tcp_speed']) < TOL
                and abs(joint - red['joint_speed']) < TOL):
            raise RuntimeError(f'modo reduzido não confirmado: ligado={on}, '
                               f'TCP={tcp} mm/s, juntas={joint} °/s')

        # Confirmação por leitura: gamas de junta dentro da especificação do
        # Lite 6 (iguais ou mais estreitas). Recusa, por exemplo, as do xArm 6.
        spec = cfg.get('lite6_joint_range')
        if spec:
            lidas = states[4][:12]
            for j in range(6):
                lo, hi = lidas[2 * j], lidas[2 * j + 1]
                if lo < spec[2 * j] - TOL or hi > spec[2 * j + 1] + TOL:
                    raise RuntimeError(
                        f'gama do modo reduzido da J{j + 1} [{lo}, {hi}] fora da '
                        f'especificação do Lite 6 {spec[2 * j:2 * j + 2]}')

        # Pergunta ao controlador (arm.error_code é uma cache que pode estar atrasada).
        code, (err, warn) = arm.get_err_warn_code()
        _check(code, 'get_err_warn_code')
        if err != 0:
            raise RuntimeError(f'controlador em erro: error_code={err}')
    except Exception:
        arm.disconnect()
        raise
    return arm


def home(arm, config_path=CONFIG):
    """Leva o braço à pose home do robot.yaml, só com o modo reduzido ligado."""
    cfg = load_config(config_path)
    code, on = arm.get_reduced_mode()
    _check(code, 'get_reduced_mode')
    if on != 1:
        raise RuntimeError('modo reduzido desligado: movimento recusado')
    _check(arm.set_servo_angle(angle=cfg['home'],
                               speed=cfg['motion']['home_speed'], wait=True),
           'set_servo_angle(home)')
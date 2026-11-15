"""Testes contra o simulador: R1-R4, P1, P2, S2."""
import numpy as np
import pytest

from lite6.session import home, load_config

CFG = load_config()
# Partidas sem risco de auto-colisão: só a base (J1) e o punho (J6) rodam.
PARTIDAS = [[30, 0, 0, 0, 0, 45], [-45, 0, 0, 0, 0, -90], [90, 0, 0, 0, 0, 30]]


def test_R1_sessao_ligada(arm):
    assert arm.connected


def test_R2_sem_erros_apos_safe_session(arm):
    code, (err, warn) = arm.get_err_warn_code()
    assert code == 0 and err == 0


def test_R3_modo_reduzido_confirmado(arm):
    code, states = arm.get_reduced_states()
    assert code == 0 and states[0] == 1
    assert states[2] == pytest.approx(CFG['reduced']['tcp_speed'], abs=0.01)
    assert states[3] == pytest.approx(CFG['reduced']['joint_speed'], abs=0.01)


def test_R4_P1_home_a_partir_de_3_poses(arm):
    finais = []
    for q in PARTIDAS:
        assert arm.set_servo_angle(angle=q, speed=20, wait=True) == 0
        home(arm)
        code, ang = arm.get_servo_angle()
        assert code == 0
        finais.append(ang[:6])
    erro = np.abs(np.array(finais) - np.array(CFG['home']))
    print(f'\nP1: erro máximo face ao home = {erro.max():.6f}°')
    assert erro.max() < 0.1


def test_P2_tempo_de_safe_session(arm):
    print(f'\nP2: safe_session demorou {arm.t_sessao:.3f} s')
    assert arm.t_sessao < 5.0


def test_S2_home_recusado_com_modo_reduzido_desligado(arm):
    assert arm.set_reduced_mode(False) == 0
    try:
        with pytest.raises(RuntimeError):
            home(arm)
    finally:
        arm.set_reduced_mode(True)
"""Exemplo 2001-move_joint (xArm 6) adaptado ao Lite 6 - LW0, Parte 1, §6.2.

Alterações face ao original:
  - usa safe_session() (limpa erros, modo reduzido confirmado, IP do robot.yaml);
  - verifica o código de retorno de todas as chamadas;
  - velocidade 20 °/s em vez de 50 °/s;
  - J3 = +60° em vez de -60°: no Lite 6 o J3 vai de -3.5° a +300°.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lite6.session import _check, home, safe_session  # noqa: E402

SPEED = 20  # °/s
POSES = [
    [90, 0, 0, 0, 0, 0],
    [90, 0, 60, 0, 0, 0],
    [90, -30, 60, 0, 0, 0],
    [0, -30, 60, 0, 0, 0],
    [0, 0, 60, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
]

arm = safe_session()
try:
    home(arm)
    for q in POSES:
        _check(arm.set_servo_angle(angle=q, speed=SPEED, wait=True),
               f'set_servo_angle({q})')
        code, ang = arm.get_servo_angle()
        _check(code, 'get_servo_angle')
        print('pedido:', q, '| lido:', [round(a, 3) for a in ang[:6]])
finally:
    arm.disconnect()
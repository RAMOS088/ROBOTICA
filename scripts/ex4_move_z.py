"""Subir 50 mm em +z com set_position e voltar.

Parte de uma pose desdobrada e afastada de singularidades (J5 != 0), mostra
quanto rodou cada junta e confirma que o TCP regressa à posição inicial.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lite6.session import _check, home, safe_session  # noqa: E402

Q0 = [0, -30, 60, 0, 30, 0]   # graus
DZ = 50                       # mm
V_TCP = 50                    # mm/s (abaixo dos 150 mm/s do modo reduzido)


def estado(arm):
    code, q = arm.get_servo_angle()
    _check(code, 'get_servo_angle')
    code, p = arm.get_position()
    _check(code, 'get_position')
    return [round(a, 3) for a in q[:6]], [round(v, 3) for v in p]


arm = safe_session()
try:
    home(arm)
    _check(arm.set_servo_angle(angle=Q0, speed=20, wait=True), 'set_servo_angle(Q0)')
    q_a, p_a = estado(arm)
    print('inicio   q =', q_a, '| pose =', p_a)

    _check(arm.set_position(z=DZ, relative=True, speed=V_TCP, wait=True),
           'set_position(+z)')
    q_b, p_b = estado(arm)
    print('+50 mm   q =', q_b, '| pose =', p_b)
    print('  variação das juntas (graus):', [round(b - a, 3) for a, b in zip(q_a, q_b)])
    print('  variação da pose:           ', [round(b - a, 3) for a, b in zip(p_a, p_b)])

    _check(arm.set_position(z=-DZ, relative=True, speed=V_TCP, wait=True),
           'set_position(-z)')
    q_c, p_c = estado(arm)
    print('regresso q =', q_c, '| pose =', p_c)
    print('  erro de regresso da pose:   ', [round(c - a, 3) for a, c in zip(p_a, p_c)])

    home(arm)
finally:
    arm.disconnect()
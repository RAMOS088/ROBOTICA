"""Regista os seis ângulos das juntas durante 30 s.

Uso:
  python3 scripts/log_joints.py --demo     simulador: comanda movimentos enquanto regista
  python3 scripts/log_joints.py --manual   robô real: modo manual (modo 2) para guiar à mão.

Gera logs/joints_<data>.csv e logs/joints_<data>.png e reporta a cadência obtida.
Cada leitura devolve a cópia local do SDK, atualizada pelo relatório do
controlador;
"""
import argparse
import csv
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

RAIZ = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(RAIZ))
from lite6.session import _check, home, safe_session  # noqa: E402

DEMO = [                                   # graus, a 20 °/s, sem auto-colisões
    [60, -30, 60, 0, 30, 45],
    [-60, -20, 90, 30, 40, -45],
    [0, -30, 60, 0, 30, 0],
    [0, 0, 0, 0, 0, 0],
]


def main():
    ap = argparse.ArgumentParser()
    modo = ap.add_mutually_exclusive_group(required=True)
    modo.add_argument('--demo', action='store_true')
    modo.add_argument('--manual', action='store_true')
    ap.add_argument('--duracao', type=float, default=30.0)
    args = ap.parse_args()

    arm = safe_session()
    t, q = [], []
    try:
        if args.demo:
            home(arm)
            for pose in DEMO:              # não bloqueante: o controlador executa em fila
                _check(arm.set_servo_angle(angle=pose, speed=20, wait=False),
                       f'set_servo_angle({pose})')
        else:
            _check(arm.set_mode(2), 'set_mode(2)')
            _check(arm.set_state(0), 'set_state(0)')
            print('Modo manual ativo: guiar o braço à mão.')

        t0 = time.perf_counter()
        while (agora := time.perf_counter() - t0) < args.duracao:
            code, ang = arm.get_servo_angle()
            _check(code, 'get_servo_angle')
            t.append(agora)
            q.append(ang[:6])
    finally:
        if args.manual:
            arm.set_mode(0)
            arm.set_state(0)
        arm.disconnect()

    # Cadência do ciclo de leitura
    dt = [b - a for a, b in zip(t, t[1:])]
    taxa = (len(t) - 1) / (t[-1] - t[0])
    print(f'{len(t)} leituras em {t[-1] - t[0]:.2f} s -> ciclo de leitura {taxa:.1f} Hz')
    print(f'intervalo entre leituras: mediana {1e3 * statistics.median(dt):.2f} ms, '
          f'mínimo {1e3 * min(dt):.2f} ms, máximo {1e3 * max(dt):.2f} ms')

    # Cadência efetiva: instantes em que chegou um valor novo
    novos = [ti for ti, a, b in zip(t[1:], q, q[1:]) if a != b]
    if len(novos) > 1:
        dn = [b - a for a, b in zip(novos, novos[1:])]
        taxa_ef = (len(novos) - 1) / (novos[-1] - novos[0])
        print(f'{len(novos)} valores novos -> cadência efetiva {taxa_ef:.1f} Hz')
        print(f'intervalo entre valores novos: mediana {1e3 * statistics.median(dn):.2f} ms, '
              f'mínimo {1e3 * min(dn):.2f} ms, máximo {1e3 * max(dn):.2f} ms')
        titulo = f'Ângulos das juntas (cadência efetiva {taxa_ef:.0f} Hz)'
    else:
        print('o braço não se mexeu: não é possível estimar a cadência efetiva')
        titulo = 'Ângulos das juntas'

    # CSV e gráfico
    nome = RAIZ / 'logs' / f'joints_{datetime.now():%Y%m%d_%H%M%S}'
    with open(nome.with_suffix('.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_s'] + [f'theta{i}_deg' for i in range(1, 7)])
        w.writerows([[f'{ti:.4f}'] + [f'{a:.3f}' for a in qi] for ti, qi in zip(t, q)])

    plt.figure(figsize=(8, 4))
    for i in range(6):
        plt.plot(t, [qi[i] for qi in q], label=rf'$\theta_{i + 1}$')
    plt.xlabel('t (s)')
    plt.ylabel('ângulo (°)')
    plt.title(titulo)
    plt.grid(True, alpha=0.3)
    plt.legend(ncol=6, fontsize=8, loc='upper right')
    plt.tight_layout()
    plt.savefig(nome.with_suffix('.png'), dpi=150)
    print(f'gravado: {nome}.csv e {nome}.png')


if __name__ == '__main__':
    main()
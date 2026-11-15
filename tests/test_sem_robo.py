"""Testes sem robô nem simulador: interface (I1) e segurança (S1, S2)."""
from pathlib import Path

import pytest

from lite6 import session

RAIZ = Path(__file__).resolve().parent.parent


class BracoFalso:
    """Imita o XArmAPI: todas as chamadas devolvem 0, exceto a indicada em `falha`."""

    def __init__(self, falha=None):
        self.falha = falha
        self.connected = True
        self.movimentos = 0
        self.desligado = False

    def _r(self, nome):
        return 1 if nome == self.falha else 0

    def clean_error(self): return self._r('clean_error')
    def clean_warn(self): return self._r('clean_warn')
    def motion_enable(self, enable=True): return self._r('motion_enable')
    def set_mode(self, m): return self._r('set_mode')
    def set_state(self, s): return self._r('set_state')
    def set_reduced_max_joint_speed(self, v): return self._r('set_reduced_max_joint_speed')
    def set_reduced_max_tcp_speed(self, v): return self._r('set_reduced_max_tcp_speed')
    def set_reduced_mode(self, on): return self._r('set_reduced_mode')

    def get_reduced_states(self):
        rng = list(session.load_config()['lite6_joint_range']) + [0, 0]
        return self._r('get_reduced_states'), [1, [0] * 6, 150.0, 54.0, rng, 0, 1]

    def get_err_warn_code(self): return self._r('get_err_warn_code'), [0, 0]
    def get_reduced_mode(self): return 0, 1

    def set_servo_angle(self, **kw):
        self.movimentos += 1
        return 0

    def disconnect(self):
        self.desligado = True


CHAMADAS = ['clean_error', 'clean_warn', 'motion_enable', 'set_mode', 'set_state',
            'set_reduced_max_joint_speed', 'set_reduced_max_tcp_speed',
            'set_reduced_mode', 'get_reduced_states', 'get_err_warn_code']


def test_I1_configuracao_completa():
    cfg = session.load_config()
    assert isinstance(cfg['ip'], str)
    assert len(cfg['home']) == 6
    assert cfg['reduced']['joint_speed'] == 54      # °/s = 30 % de 180 °/s
    assert cfg['reduced']['tcp_speed'] == 150       # mm/s
    assert len(cfg['lite6_joint_range']) == 12      # min e max de J1..J6


def test_I1_sem_ip_no_codigo():
    for py in list((RAIZ / 'lite6').glob('*.py')) + list((RAIZ / 'scripts').glob('*.py')):
        assert '192.168' not in py.read_text(), f'IP escrito em {py.name}'


@pytest.mark.parametrize('falha', CHAMADAS)
def test_S1_codigo_nao_nulo_levanta_erro_sem_movimento(monkeypatch, falha):
    falso = BracoFalso(falha)
    monkeypatch.setattr(session, 'XArmAPI', lambda *a, **k: falso)
    with pytest.raises(RuntimeError):
        session.safe_session()
    assert falso.movimentos == 0
    assert falso.desligado


def test_S2_modo_reduzido_nao_confirmado_e_recusado(monkeypatch):
    falso = BracoFalso()
    falso.get_reduced_states = lambda: (0, [1, [0] * 6, 250.0, 54.0])   # TCP errado
    monkeypatch.setattr(session, 'XArmAPI', lambda *a, **k: falso)
    with pytest.raises(RuntimeError):
        session.safe_session()


def test_S2_home_recusado_sem_modo_reduzido():
    falso = BracoFalso()
    falso.get_reduced_mode = lambda: (0, 0)
    with pytest.raises(RuntimeError):
        session.home(falso)
    assert falso.movimentos == 0


def test_S2_gamas_do_xarm6_recusadas(monkeypatch):
    falso = BracoFalso()
    xarm6 = [-360, 360, -118, 120, -225, 11, -360, 360, -97, 180, -360, 360, 0, 0]
    falso.get_reduced_states = lambda: (0, [1, [0] * 6, 150.0, 54.0, xarm6, 0, 1])
    monkeypatch.setattr(session, 'XArmAPI', lambda *a, **k: falso)
    with pytest.raises(RuntimeError):
        session.safe_session()
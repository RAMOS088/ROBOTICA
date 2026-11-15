"""Preparação comum dos testes: caminho do pacote e sessão no simulador."""
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from lite6.session import safe_session  # noqa: E402


@pytest.fixture(scope='module')
def arm():
    t0 = time.perf_counter()
    try:
        a = safe_session()
    except RuntimeError as e:
        if 'sem ligação' in str(e):
            pytest.skip(f'simulador indisponível: {e}')
        raise
    a.t_sessao = time.perf_counter() - t0
    yield a
    a.disconnect()
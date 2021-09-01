# content of test_sample.py
from smartdrone.core import raise_exception
import pytest

from smartdrone.core import func

def test_answer():
    y = 5
    import pdb;pdb.set_trace()
    x = y-3
    assert func(x) == 5

@pytest.mark.slow
def test_raise_exception():
    with pytest.raises(SystemExit):
        raise_exception()
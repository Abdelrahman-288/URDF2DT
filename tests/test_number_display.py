import pytest
from urdf2dt.ui.number_display import decimal, residual, display


@pytest.mark.parametrize("value,expected", [(1.9249999999999998,"1.925"),(.010999999999999982,"0.011"),
    (1.57079632679,"1.571"),(-0.0,"0.000"),(-1e-10,"0.000"),(-.1236,"-0.124")])
def test_three_decimals(value,expected):
    assert decimal(value)==expected


def test_small_errors_visible_and_vectors_unchanged():
    values=[1.23456789,-1e-8]
    assert display(values)=='[1.235, 0.000]'
    assert values==[1.23456789,-1e-8]
    assert residual(1.234e-12)=='1.234e-12'
    assert residual(0)=='0.000'

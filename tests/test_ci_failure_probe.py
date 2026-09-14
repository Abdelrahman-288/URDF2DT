"""Temporary Stage 17 check: intentionally demonstrate a red GitHub run."""


def test_intentional_ci_failure():
    assert False, "Intentional Stage 17 failure probe; removed after CI detects it"

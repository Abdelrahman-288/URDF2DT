"""Adapter boundary, source preservation, reproducibility and CLI behavior."""

from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys

import pytest

from urdf2dt.config import EditorConfig
from urdf2dt.dh.dh_solver import StandardDHSolver
from urdf2dt.parser.urdf_parser import SerialURDFParser
from urdf2dt.parser.urdf_validator import URDFValidationError
from urdf2dt.pipeline import generate_automatic_model

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "robots/ur5/ur5_serial.urdf"


def test_invalid_input_never_reaches_adapters():
    class Never:
        def parse(self, *args):
            pytest.fail("invalid input reached parser")
        def solve(self, *args):
            pytest.fail("invalid input reached solver")
    with pytest.raises(URDFValidationError):
        generate_automatic_model(ROOT / "tests/fixtures/invalid_urdf/xxe.urdf", parser=Never(), solver=Never())


def test_parser_adapter_identity_checked():
    class WrongParser:
        def parse(self, source, config):
            chain = SerialURDFParser().parse(source, config)
            return replace(chain, source_sha256="0" * 64)
    with pytest.raises(ValueError, match="parser adapter"):
        generate_automatic_model(SOURCE, parser=WrongParser())


def test_solver_adapter_identity_checked():
    class WrongSolver:
        def solve(self, chain, config):
            model = StandardDHSolver().solve(chain, config)
            return replace(model, rows=tuple(reversed(model.rows)))
    with pytest.raises(ValueError, match="solver adapter"):
        generate_automatic_model(SOURCE, solver=WrongSolver())


def test_repeated_generation_is_deterministic_and_retains_config():
    config = EditorConfig()
    first = generate_automatic_model(SOURCE, config)
    second = generate_automatic_model(SOURCE, config)
    assert first == second
    assert first.config is config
    assert first.chain.source_sha256 == first.automatic_model.source_sha256


@pytest.mark.parametrize("source,exit_code", [(SOURCE, 0), (ROOT / "tests/fixtures/invalid_urdf/xxe.urdf", 1)])
def test_installed_cli_in_isolated_directory(tmp_path, source, exit_code):
    process = subprocess.run([sys.executable, "-I", "-m", "urdf2dt", str(source), "--json"],
                             cwd=tmp_path, capture_output=True, text=True)
    assert process.returncode == exit_code, process.stderr
    output = json.loads(process.stdout)
    assert output["generated"] == (exit_code == 0)
    if exit_code == 0:
        assert output["globally_validated"] is False
        assert len(output["model"]["rows"]) == 6
        assert output["model"]["is_temporary_fixture"] is False

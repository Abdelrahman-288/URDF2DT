"""Resource and provenance regressions for standalone Windows builds."""

import json
import sys

from urdf2dt import runtime
from urdf2dt.logging_config import git_provenance


def test_source_assets_do_not_depend_on_cwd(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    assert runtime.asset_path("robots/scara/scara_rrpr.urdf").is_file()
    assert runtime.asset_path("assets/urdf2dt.svg").is_file()


def test_frozen_provenance_uses_embedded_record_without_git(monkeypatch, tmp_path):
    monkeypatch.setattr(sys, "frozen", True, raising=False)
    monkeypatch.setattr(sys, "_MEIPASS", str(tmp_path), raising=False)
    resources = tmp_path / "resources"
    resources.mkdir()
    (resources / "build-info.json").write_text(json.dumps({
        "git_commit": "a" * 40, "working_tree_clean": True}), encoding="utf-8")
    monkeypatch.setattr("subprocess.check_output", lambda *a, **k: (_ for _ in ()).throw(AssertionError("Git invoked")))
    assert git_provenance() == {"git_commit": "a" * 40, "working_tree_clean": True}
    assert runtime.asset_path("config/default.yaml") == resources / "config/default.yaml"


def test_missing_frozen_metadata_does_not_use_unrelated_checkout(monkeypatch, tmp_path):
    monkeypatch.setattr(sys, "frozen", True, raising=False)
    monkeypatch.setattr(sys, "_MEIPASS", str(tmp_path), raising=False)
    assert git_provenance() == {"git_commit": None, "working_tree_clean": None}

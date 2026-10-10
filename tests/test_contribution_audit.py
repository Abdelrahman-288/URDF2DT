"""Exercise contribution auditing against real, isolated Git histories."""
import importlib.util
from pathlib import Path
import subprocess

import pytest

spec = importlib.util.spec_from_file_location(
    "contribution_audit", Path(__file__).parents[1] / "scripts/contribution_audit.py")
assert spec is not None and spec.loader is not None
audit_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit_module)


@pytest.fixture
def repo(tmp_path, monkeypatch):
    # Keep global identity and environment overrides out of the synthetic history.
    for key in ("GIT_AUTHOR_NAME", "GIT_AUTHOR_EMAIL", "GIT_COMMITTER_NAME", "GIT_COMMITTER_EMAIL"):
        monkeypatch.delenv(key, raising=False)
    def git(*args):
        return subprocess.check_output(["git", "-C", str(tmp_path), *args], text=True).strip()
    git("init", "-b", "main")
    git("config", "user.name", "Audit Test")
    git("config", "user.email", "audit@example.invalid")
    (tmp_path / "source.txt").write_text("initial")
    git("add", "source.txt")
    git("commit", "-m", "Initial fixture")
    git("update-ref", "refs/remotes/origin/main", "HEAD")
    git("symbolic-ref", "refs/remotes/origin/HEAD", "refs/remotes/origin/main")
    return tmp_path, git


def test_local_only_work_and_dirty_files_are_reported_without_mutation(repo):
    path, git = repo
    git("checkout", "-b", "feature")
    (path / "source.txt").write_text("feature")
    git("add", "source.txt")
    git("commit", "-m", "Feature fixture")
    (path / "source.txt").write_text("unfinished")
    (path / "notes.txt").write_text("untracked")
    before = git("status", "--porcelain=v1")
    result = audit_module.audit(path, "audit@example.invalid")
    assert len(result["local_commits_not_on_any_remote"]) == 1
    feature = next(b for b in result["branches"] if b["branch"] == "feature")
    assert feature["not_on_default"] == 1
    assert feature["ahead_of_upstream"] is None
    assert any("notes.txt" in s for s in result["status"])
    assert "feature" in result["refs_not_merged_into_default"]
    assert git("status", "--porcelain=v1") == before
    # A branch can be unpushed to its old upstream but already included in main.
    git("update-ref", "refs/remotes/origin/main", "feature")
    assert not audit_module.audit(path)["local_commits_not_on_any_remote"]


def test_environment_identity_and_unknown_default(repo, monkeypatch):
    path, git = repo
    monkeypatch.setenv("GIT_AUTHOR_EMAIL", "override@example.invalid")
    git("symbolic-ref", "--delete", "refs/remotes/origin/HEAD")
    result = audit_module.audit(path, "audit@example.invalid")
    assert "override@example.invalid" in result["author_identity"]
    assert any("differs" in w for w in result["warnings"])
    assert result["default_ref"] is None
    assert result["refs_not_merged_into_default"] is None
    git("checkout", "--detach")
    assert audit_module.audit(path)["branch"] == "(detached HEAD)"

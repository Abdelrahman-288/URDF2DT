"""File selection and upload handling use immutable, size-limited snapshots."""

from dataclasses import FrozenInstanceError
from hashlib import sha256
from pathlib import Path

import pytest

from urdf2dt.parser.urdf_input import InputPolicy, URDFInput, URDFInputError
from urdf2dt.parser.urdf_validator import validate_urdf


def test_upload_owns_bytes():
    data = bytearray(b"<robot/>")
    source = URDFInput.from_upload("robot.URDF", memoryview(data))
    data[0] = 0
    assert source.content == b"<robot/>"
    assert source.sha256 == sha256(b"<robot/>").hexdigest()
    with pytest.raises(FrozenInstanceError):
        source.name = "other.urdf"


def test_file_snapshot_does_not_change_when_original_changes(tmp_path):
    path = tmp_path / "robot.urdf"
    path.write_bytes(b"<robot/>")
    source = URDFInput.from_path(path)
    path.write_bytes(b"changed")
    assert source.content == b"<robot/>"
    assert source.source_path == str(path.resolve())


@pytest.mark.parametrize("selection,code", [(None, "selection_cancelled"), ("", "selection_cancelled"),
                                             ("x.xacro", "invalid_extension"), ("a\x00.urdf", "invalid_filename")])
def test_invalid_selection(selection, code):
    result = validate_urdf(selection)
    assert not result.valid
    assert result.errors[0].code == code


def test_missing_directory_empty_and_unreadable(tmp_path, monkeypatch):
    path = tmp_path / "robot.urdf"
    assert validate_urdf(path).errors[0].code == "file_not_found"
    path.mkdir()
    assert validate_urdf(path).errors[0].code == "not_a_file"
    other = tmp_path / "empty.urdf"
    other.write_bytes(b"")
    assert validate_urdf(other).errors[0].code == "empty_file"
    def denied(*args, **kwargs):
        raise PermissionError("test permission denial")
    monkeypatch.setattr(Path, "open", denied)
    assert validate_urdf(other).errors[0].code == "file_unreadable"


def test_input_size_bounds_apply_to_every_entry_point(tmp_path):
    policy = InputPolicy(max_bytes=8)
    assert URDFInput.from_upload("a.urdf", b"12345678", policy).content == b"12345678"
    with pytest.raises(URDFInputError) as exc:
        URDFInput.from_upload("a.urdf", b"123456789", policy)
    assert exc.value.issue.code == "file_too_large"
    assert validate_urdf(URDFInput("a.urdf", b"123456789"), policy).errors[0].code == "file_too_large"
    path = tmp_path / "a.urdf"
    path.write_bytes(b"123456789")
    assert validate_urdf(path, policy).errors[0].code == "file_too_large"


def test_read_is_bounded(tmp_path, monkeypatch):
    from io import BytesIO
    path = tmp_path / "a.urdf"
    path.write_bytes(b"placeholder")
    sizes = []
    class Reader(BytesIO):
        def read(self, size=-1):
            sizes.append(size)
            return super().read(size)
    monkeypatch.setattr(Path, "open", lambda *a, **kw: Reader(b"x" * 100))
    assert validate_urdf(path, InputPolicy(max_bytes=10)).errors[0].code == "file_too_large"
    assert sizes == [11]


@pytest.mark.parametrize("field,value", [("max_bytes", 0), ("max_depth", -1), ("max_elements", True)])
def test_bad_resource_policy(field, value):
    with pytest.raises(ValueError):
        InputPolicy(**{field: value})

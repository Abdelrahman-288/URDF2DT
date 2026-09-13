"""Bounded file/upload input without a notebook or native file-dialog dependency."""

from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path


@dataclass(frozen=True, slots=True)
class URDFIssue:
    """Stable machine-readable code plus a user-facing message and XML context."""

    code: str
    message: str
    element: str | None = None


class URDFInputError(ValueError):
    """Input failure available to both headless callers and file-selection UIs."""

    def __init__(self, issue: URDFIssue):
        self.issue = issue
        super().__init__(issue.message)


@dataclass(frozen=True, slots=True)
class InputPolicy:
    """Resource bounds, separate from provisional geometry/FK tolerances."""

    max_bytes: int = 5 * 1024 * 1024
    max_elements: int = 20000
    max_depth: int = 100

    def __post_init__(self) -> None:
        for name in ("max_bytes", "max_elements", "max_depth"):
            value = getattr(self, name)
            if type(value) is not int or value < 1:
                raise ValueError(f"{name} must be a positive integer")


@dataclass(frozen=True, slots=True)
class URDFInput:
    """Owned XML bytes; the same snapshot is validated and later parsed.

    source_path is provenance only. Upload names are labels, never write targets.
    Reading does not validate XML; use validate_urdf before kinematic parsing.
    """

    name: str
    content: bytes
    source_path: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise URDFInputError(URDFIssue("invalid_filename", "Select a named .urdf file."))
        if not isinstance(self.content, (bytes, bytearray, memoryview)):
            raise TypeError("URDF content must be bytes, bytearray or memoryview")
        object.__setattr__(self, "content", bytes(self.content))

    @property
    def sha256(self) -> str:
        """Digest of the exact selected bytes, independent of later file changes."""
        return sha256(self.content).hexdigest()

    @classmethod
    def from_path(cls, path: str | Path | None, policy: InputPolicy = InputPolicy()) -> URDFInput:
        """Read a selection once; None/empty string represents a cancelled dialog."""
        if path is None or (isinstance(path, str) and not path.strip()):
            raise URDFInputError(URDFIssue("selection_cancelled", "No URDF file was selected."))
        selected = Path(path).expanduser()
        _check_name(selected.name)
        try:
            if not selected.exists():
                raise URDFInputError(URDFIssue("file_not_found", f"URDF file not found: {selected}"))
            if not selected.is_file():
                raise URDFInputError(URDFIssue("not_a_file", f"Select a regular file: {selected}"))
            with selected.open("rb") as stream:
                content = stream.read(policy.max_bytes + 1)
            source = str(selected.resolve())
        except OSError as exc:
            raise URDFInputError(URDFIssue("file_unreadable", f"Cannot read URDF file: {selected}")) from exc
        _check_size(len(content), policy)
        return cls(selected.name, content, source)

    @classmethod
    def from_upload(cls, name: str, content: bytes | bytearray | memoryview,
                    policy: InputPolicy = InputPolicy()) -> URDFInput:
        """Adapt a widget's filename/content pair without saving or executing it."""
        _check_name(name)
        _check_size(content.nbytes if isinstance(content, memoryview) else len(content), policy)
        return cls(name, bytes(content))


def _check_name(name: str) -> None:
    if not isinstance(name, str) or not name.strip() or "\x00" in name:
        raise URDFInputError(URDFIssue("invalid_filename", "Select a named .urdf file."))
    if Path(name).suffix.lower() != ".urdf":
        raise URDFInputError(URDFIssue("invalid_extension", "Select a .urdf file; expand Xacro files first."))


def _check_size(size: int, policy: InputPolicy) -> None:
    if size == 0:
        raise URDFInputError(URDFIssue("empty_file", "The selected URDF file is empty."))
    if size > policy.max_bytes:
        raise URDFInputError(URDFIssue("file_too_large", f"URDF exceeds the {policy.max_bytes}-byte input limit."))

"""Strict, immutable configuration; defaults are provisional research inputs."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from math import isfinite
from pathlib import Path
from typing import Any, Mapping

import yaml


class ConfigError(ValueError):
    """A configuration contains invalid, missing, duplicate or unknown values."""


def _positive(value: float, name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{name} must be a finite positive number")
    if not isfinite(value) or value <= 0:
        raise ConfigError(f"{name} must be a finite positive number")


@dataclass(frozen=True, slots=True)
class GeometryConfig:
    """Parallel threshold is dimensionless; distance thresholds are meters."""

    parallel_threshold: float = 1.0e-5
    intersection_threshold: float = 1.0e-6
    common_normal_threshold: float = 1.0e-6

    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__:
            _positive(getattr(self, name), name)
        if self.parallel_threshold >= 1:
            raise ConfigError("parallel_threshold must be less than 1")


@dataclass(frozen=True, slots=True)
class ValidationConfig:
    """FK tolerances and deterministic sampling settings (including zero pose)."""

    position_tolerance_m: float = 1.0e-4
    orientation_tolerance_rad: float = 1.0e-4
    samples: int = 50
    random_seed: int = 42

    def __post_init__(self) -> None:
        _positive(self.position_tolerance_m, "position_tolerance_m")
        _positive(self.orientation_tolerance_rad, "orientation_tolerance_rad")
        if type(self.samples) is not int or self.samples < 1:
            raise ConfigError("samples must be a positive integer")
        if type(self.random_seed) is not int or self.random_seed < 0:
            raise ConfigError("random_seed must be a nonnegative integer")


@dataclass(frozen=True, slots=True)
class UIConfig:
    """UI preferences without importing the UI runtime."""

    live_preview: bool = True

    def __post_init__(self) -> None:
        if type(self.live_preview) is not bool:
            raise ConfigError("live_preview must be a boolean")


@dataclass(frozen=True, slots=True)
class LoggingConfig:
    """Supported standard Python logging level names."""

    level: str = "INFO"

    def __post_init__(self) -> None:
        if self.level not in ("DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"):
            raise ConfigError("unsupported logging level")


@dataclass(frozen=True, slots=True)
class EditorConfig:
    """Effective values plus an explicit record of their reference status."""

    geometry: GeometryConfig = field(default_factory=GeometryConfig)
    validation: ValidationConfig = field(default_factory=ValidationConfig)
    ui: UIConfig = field(default_factory=UIConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)
    reference_status: str = "provisional"
    reference_note: str = "Plan section 10; advisor/report reconciliation pending."

    def __post_init__(self) -> None:
        for name, expected in (
            ("geometry", GeometryConfig), ("validation", ValidationConfig),
            ("ui", UIConfig), ("logging", LoggingConfig),
        ):
            if not isinstance(getattr(self, name), expected):
                raise ConfigError(f"{name} must be {expected.__name__}")
        if self.reference_status not in ("provisional", "confirmed"):
            raise ConfigError("reference_status must be provisional or confirmed")
        if not isinstance(self.reference_note, str) or not self.reference_note.strip():
            raise ConfigError("reference_note must explain threshold provenance")

    def snapshot(self) -> dict[str, Any]:
        """Return an independent, JSON-compatible copy of every effective value."""
        return asdict(self)


class _UniqueSafeLoader(yaml.SafeLoader):
    """Reject duplicate keys (including those introduced through YAML merges)."""

    def construct_mapping(self, node: yaml.MappingNode, deep: bool = False) -> Any:
        """Reject duplicate YAML mapping keys before constructing configuration values."""
        self.flatten_mapping(node)
        seen: set[str] = set()
        for key_node, _ in node.value:
            key = self.construct_object(key_node, deep=deep)
            if not isinstance(key, str):
                raise ConfigError("configuration keys must be strings")
            if key in seen:
                raise ConfigError(f"duplicate configuration key: {key}")
            seen.add(key)
        return super().construct_mapping(node, deep=deep)


def config_from_mapping(data: Mapping[str, Any]) -> EditorConfig:
    """Load a complete mapping, refusing misspellings and silent default fallback."""
    classes: dict[str, type[GeometryConfig] | type[ValidationConfig] | type[UIConfig] | type[LoggingConfig]] = {
        "geometry": GeometryConfig, "validation": ValidationConfig,
        "ui": UIConfig, "logging": LoggingConfig,
    }
    required = set(classes) | {"reference_status", "reference_note"}
    if set(data) != required:
        raise ConfigError(f"expected top-level keys: {', '.join(sorted(required))}")
    values: dict[str, Any] = {}
    for section, cls in classes.items():
        raw = data[section]
        if not isinstance(raw, Mapping) or set(raw) != set(cls.__dataclass_fields__):
            raise ConfigError(f"{section} has missing or unknown fields")
        values[section] = cls(**raw)
    return EditorConfig(**values, reference_status=data["reference_status"],
                        reference_note=data["reference_note"])


def load_config(path: str | Path | None = None) -> EditorConfig:
    """Load a complete YAML file, or typed built-in defaults independent of cwd.

    Filesystem errors are preserved. YAML syntax and value errors are ConfigError.
    The checked-in YAML must match the built-in defaults; tests enforce this.
    """
    if path is None:
        return EditorConfig()
    try:
        data = yaml.load(Path(path).read_text(encoding="utf-8-sig"), Loader=_UniqueSafeLoader)
    except yaml.YAMLError as exc:
        raise ConfigError(f"invalid YAML: {exc}") from exc
    if not isinstance(data, Mapping):
        raise ConfigError("configuration must be a mapping")
    return config_from_mapping(data)

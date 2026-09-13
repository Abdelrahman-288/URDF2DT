"""Generate an automatic DH baseline: python -m urdf2dt robot.urdf [--json]."""

import argparse
from dataclasses import asdict
import json

from urdf2dt.config import ConfigError, load_config
from urdf2dt.dh.classification import classify_dh_model, get_editable_params
from urdf2dt.dh.dh_solver import DHSolverError
from urdf2dt.parser.urdf_validator import URDFValidationError
from urdf2dt.pipeline import generate_automatic_model


def main() -> int:
    """Run Stage 5 only; no editing or validated-session export is implied."""
    parser = argparse.ArgumentParser(description="Generate an automatic Standard-DH baseline from serial URDF.")
    parser.add_argument("path")
    parser.add_argument("--config", help="Complete YAML configuration; defaults are provisional")
    parser.add_argument("--json", action="store_true", help="Print a development summary, not a validated export")
    parser.add_argument("--classify", action="store_true", help="Include zero-pose geometric cases and local edit freedoms")
    args = parser.parse_args()
    try:
        result = generate_automatic_model(args.path, load_config(args.config))
    except (URDFValidationError, DHSolverError, ConfigError, OSError) as exc:
        if args.json:
            errors = ([asdict(issue) for issue in exc.errors] if isinstance(exc, URDFValidationError)
                      else [{"code": getattr(exc, "code", "configuration_error"), "message": str(exc)}])
            print(json.dumps({"generated": False, "errors": errors}, indent=2))
        else:
            print(str(exc))
        return 1
    model = result.automatic_model
    cases = classify_dh_model(model, result.config.geometry) if args.classify else ()
    if args.json:
        print(json.dumps({
            "generated": True, "globally_validated": False,
            "model": asdict(model), "configuration": result.config.snapshot(),
            "warnings": [asdict(issue) for issue in result.warnings],
            **({"axis_classifications": [dict(asdict(case), frame=f"F{i}",
                 terminal_convention=i == len(cases),
                 editable_params=[asdict(p) for p in get_editable_params(case)])
                 for i, case in enumerate(cases, 1)]} if args.classify else {}),
        }, indent=2, allow_nan=False))
    else:
        print(f"Automatic Standard-DH model: {model.robot_name}; {len(model.rows)} movable joints")
        print("Joint                         a [m]      alpha [rad]   d [m]      theta offset [rad]")
        for row in model.rows:
            print(f"{row.joint_name:28} {row.a: .9g}  {row.alpha: .9g}  {row.d: .9g}  {row.theta_offset: .9g}")
        print(f"Reference status: {result.config.reference_status}; global validation has not run.")
        print("The model includes base/tool transforms; use --json to inspect them.")
        for warning in result.warnings:
            print(f"Warning [{warning.code}]: {warning.message}")
        for i, case in enumerate(cases, 1):
            controls = ", ".join(p.name for p in get_editable_params(case)) or "none"
            print(f"F{i}: {case.case.value}; distance={case.distance_m:.6g} m; controls={controls}")
            print(f"  {case.description}")
        if cases:
            print("Final pair uses the terminal DH axis convention. A/B1/B2 reference mapping remains unverified.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

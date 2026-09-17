"""Run validated dynamics, archive its parameters and record independent evidence."""

import argparse
import json
from pathlib import Path
from urdf2dt.dynamics.io import export_model, load_dynamic_model
from urdf2dt.dynamics.validation import ValidationSettings, validate_dynamics
from urdf2dt.parser.urdf_input import URDFInput


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source")
    parser.add_argument("--output", required=True, type=Path, help="New output directory")
    parser.add_argument("--samples", type=int, default=64)
    parser.add_argument("--seed", type=int, default=2401)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Output directory already exists; choose a new directory")
    try:
        source = URDFInput.from_path(args.source)
        model = load_dynamic_model(source)
        report = validate_dynamics(model, source, ValidationSettings(args.samples, args.seed))
        args.output.mkdir(parents=True, exist_ok=False)
        (args.output / "validation.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        if not report["passed"]:
            print("FAIL: independent dynamics comparison; see", args.output / "validation.json")
            return 1
        export_model(model, source, args.output / "dynamic_model.json")
        print(f"PASS: {model.chain.robot_name}, {model.dof} joints; results: {args.output}")
        return 0
    except ImportError as exc:
        parser.exit(2, f"Install the dynamics-reference extra for independent checks: {exc}\n")
    except (ValueError, OSError) as exc:
        parser.exit(2, f"Dynamics unavailable: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())

"""Prepare Stage 27 controller configurations and sensitivity evidence."""

import argparse
import json
from .experiments import run_experiment


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("robot", choices=("ur5", "scara"))
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    report = run_experiment(args.robot, args.output)
    print(json.dumps({"robot": report["robot"], "cases": len(report["cases"]),
                      "period_s": report["period_s"], "scope": report["scope"]}, indent=2))


if __name__ == "__main__":
    main()

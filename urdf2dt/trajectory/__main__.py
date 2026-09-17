"""Generate Stage 26 demonstrations and versioned controller reference files."""

import argparse
import json
from .demo import run_demo


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("robot", choices=("ur5", "scara"))
    parser.add_argument("--output", required=True)
    parser.add_argument("--identified-parameters", help="Optional matching Stage 25 parameter archive")
    args = parser.parse_args()
    report = run_demo(args.robot, args.output, args.identified_parameters)
    print(json.dumps({"passed": report["passed"], "robot": report["robot"],
                      "durations_s": {k: v["duration_s"] for k, v in report["trajectories"].items()}}, indent=2))


if __name__ == "__main__":
    main()

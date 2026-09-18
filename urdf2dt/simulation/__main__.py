"""Run closed-loop benchmarks with convergence and independent-reference checks."""

import argparse
import json
from .study import run_study


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("robot",choices=("ur5","scara"))
    parser.add_argument("--output",required=True)
    parser.add_argument("--plots",action="store_true",help="Save nominal tracking/effort PNG (requires research extra)")
    args = parser.parse_args()
    report = run_study(args.robot,args.output)
    if args.plots and len(report["cases"]) == 9:
        from .plot import plot_study
        plot_study(args.output)
    print(json.dumps({"passed":report["passed"],"cases":len(report["cases"]),"failures":report["failures"],
                      "comparisons":[{"mode":c["mode"],"checks":c["checks"]} for c in report["comparisons"]]},indent=2))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

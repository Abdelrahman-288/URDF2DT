"""Headless identification commands; no hardware motion or UI dependency."""

import argparse
import json
from pathlib import Path

from urdf2dt.parser.urdf_input import URDFInput
from .data import IdentificationData, require_held_out, write_record
from .fit import identify, evaluate
from .io import source_regressor, export_parameters, trust_manufacturer


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    benchmark = commands.add_parser("benchmark", help="Independent synthetic parameter-recovery study")
    benchmark.add_argument("urdf")
    benchmark.add_argument("--output", required=True)
    benchmark.add_argument("--samples", type=int, default=240)
    fit = commands.add_parser("fit", help="Fit versioned measured or simulated datasets")
    fit.add_argument("urdf")
    fit.add_argument("training")
    fit.add_argument("held_out")
    fit.add_argument("--output", required=True)
    fit.add_argument("--massless-link", action="append", default=[])
    fit.add_argument("--gravity", type=float, nargs=3, default=(0., 0., -9.81))
    trusted = commands.add_parser("trust", help="Explicit manufacturer-parameter acceptance, without fitting")
    trusted.add_argument("urdf")
    trusted.add_argument("--output", required=True)
    trusted.add_argument("--provenance", required=True)
    args = parser.parse_args()
    if args.command == "benchmark":
        from .benchmark import run_benchmark
        report = run_benchmark(args.urdf, args.output, args.samples)
        print(json.dumps({"passed": report["passed"], "held_out": report["held_out"],
                          "identifiable_combination_relative_error": report["identifiable_combination_relative_error"]}, indent=2))
    elif args.command == "trust":
        print(trust_manufacturer(URDFInput.from_path(args.urdf), args.output, args.provenance))
    else:
        source = URDFInput.from_path(args.urdf)
        regressor = source_regressor(source, args.gravity, tuple(args.massless_link))
        train, held = IdentificationData.load(args.training), IdentificationData.load(args.held_out)
        require_held_out(train, held)
        held.validate_chain(regressor.chain)
        baseline = regressor.generic_prior()
        result = identify(regressor, train)
        report = evaluate(regressor, result, train, held, baseline)
        report["baseline_description"] = "Generic 1kg/.02kg*m² prior; not manufacturer data"
        output = Path(args.output)
        output.mkdir(parents=True, exist_ok=False)
        export_parameters(output/"parameters.json", source, regressor, result)
        write_record(output/"evaluation.json", "urdf2dt.identification_evaluation", report)
        print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()

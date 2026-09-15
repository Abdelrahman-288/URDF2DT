"""Run and plot Stage 14 research, preserving source/configuration/code identities."""

import argparse
from datetime import datetime, timezone
from importlib.metadata import version
from pathlib import Path
import platform
import subprocess

from urdf2dt.parser.urdf_input import URDFInput
from urdf2dt.research.ablations import code_fingerprint, run_studies, write_tables
from urdf2dt.parser.robot_document import RobotDocument


def plot_studies(result, folder):
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    groups = sorted({(r["frame"], r["parameter"]) for r in result["sweep"]})
    fig, axes = plt.subplots(
        2,
        len(groups),
        figsize=(4 * len(groups), 7),
        squeeze=False,
        constrained_layout=True,
    )
    for col, (frame, parameter) in enumerate(groups):
        rows = [
            r
            for r in result["sweep"]
            if (r["frame"], r["parameter"]) == (frame, parameter)
            and r["outcome"] == "evaluated"
        ]
        for axis, metric, label, color in (
            (axes[0, col], "max_position_error_m", "Position error [m]", "#2364aa"),
            (
                axes[1, col],
                "max_orientation_error_rad",
                "Orientation error [rad]",
                "#ae4969",
            ),
        ):
            axis.plot(
                [r["value"] for r in rows],
                [r[metric] for r in rows],
                "o-",
                color=color,
                markersize=3,
            )
            axis.set(
                xlabel=f"{parameter.replace('_',' ')} [{rows[0]['unit'] if rows else '?'}]",
                ylabel=label,
            )
            axis.ticklabel_format(axis="y", style="sci", scilimits=(0, 0))
            axis.grid(alpha=0.2)
        axes[0, col].set_title(f"F{frame} • {parameter.replace('axial_','')}")
    fig.suptitle("Independent legal edits: maximum error over 50 fixed joint poses")
    fig.savefig(folder / "edit_sweep.png", dpi=160)
    plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), layout="constrained")
    for axis, metric, label in (
        (axes[0], "max_position_error_m", "Position error [m]"),
        (axes[1], "max_orientation_error_rad", "Orientation error [rad]"),
    ):
        for model in ("automatic", "edited", "negative_control"):
            rows = [r for r in result["density"] if r["model"] == model]
            axis.plot(
                [r["samples"] for r in rows],
                [r[metric] for r in rows],
                "o-",
                label=model,
            )
        axis.set(
            xlabel="Joint poses (nested samples, seed 42)",
            ylabel=label,
            xticks=[10, 50, 200],
        )
        axis.ticklabel_format(axis="y", style="sci", scilimits=(0, 0))
        axis.grid(alpha=0.2)
    axes[0].set_yscale("log")
    axes[0].legend(fontsize=8)
    fig.suptitle("Sample-density study (negative control: +1 mm DH link length)")
    fig.savefig(folder / "sample_density.png", dpi=160)
    plt.close(fig)


def summary(result):
    lines = [
        "# Stage 14: Research ablation results",
        "",
        f"Source: `{result['source']['name']}`; chain `{result['source']['base']} → {result['source']['tip']}`.",
        "",
        f"Selected source SHA-256: `{result['source']['sha256']}`.",
        "",
        "Full settings, source/code fingerprints, sampled poses and errors are in `results.json`.",
        "",
        "## Threshold sensitivity",
        "",
        "The same automatic model is reclassified at 0.1, 0.3, 1, 3 and 10 times the default parallel threshold. Other thresholds stay fixed.",
        "",
        "| Parallel threshold | Changed frames |",
        "|---:|---|",
    ]
    for row in result["thresholds"]:
        lines.append(
            f"| {row['configuration']['parallel_threshold']:.3g} | {row['changed_frames'] or 'None'} |"
        )
    lines += [
        "",
        "## Legal edit sweeps",
        "",
        "Each point starts from the automatic model and traverses the real acceptance gates. Translation is tested over [-1,1] m; rotation over one period [-pi,pi]. These are finite samples, not full-domain coverage.",
        "",
        "| Frame / freedom | Evaluated / total | Local + FK passed | Max position [m] | Max orientation [rad] |",
        "|---|---:|---:|---:|---:|",
    ]
    for frame, parameter in sorted(
        {(r["frame"], r["parameter"]) for r in result["sweep"]}
    ):
        rows = [
            r
            for r in result["sweep"]
            if (r["frame"], r["parameter"]) == (frame, parameter)
        ]
        evaluated = [r for r in rows if r["outcome"] == "evaluated"]
        p = max((r["max_position_error_m"] for r in evaluated), default=float("nan"))
        a = max(
            (r["max_orientation_error_rad"] for r in evaluated), default=float("nan")
        )
        lines.append(
            f"| F{frame} {parameter} | {len(evaluated)} / {len(rows)} | {sum(r['local_valid'] and r['passed'] for r in rows)} | {p:.6g} | {a:.6g} |"
        )
    lines += [
        "",
        "## Sample density",
        "",
        "Zero pose is included. Seed 42 gives nested 10/50/200 sample sets. The edited model combines the explicitly recorded legal edits. The negative control deliberately bypasses acceptance and is never a validated export.",
        "",
        "| Model | Samples | FK pass | Max position [m] | Max orientation [rad] |",
        "|---|---:|---|---:|---:|",
    ]
    for r in result["density"]:
        lines.append(
            f"| {r['model']} | {r['samples']} | {r['passed']} | {r['max_position_error_m']:.6g} | {r['max_orientation_error_rad']:.6g} |"
        )
    lines += [
        "",
        "## Synthetic boundaries",
        "",
        "| Family | Factor | Case | Coincident | Review required | Editable freedoms |",
        "|---|---:|---|---|---|---|",
    ]
    for r in result["boundary"]:
        if r["factor"] in (0.999, 1.0, 1.001):
            lines.append(
                f"| {r['family']} | {r['factor']} | {r['case']} | {r['coincident']} | {r['requires_review']} | {', '.join(r['editable_parameters']) or 'None'} |"
            )
    lines += [
        "",
        "## Interpretation and limits",
        "",
        "- Boundary changes are intentional: normalized cross magnitude and shortest distance determine the recorded cases. Approximate parallel/coincident cases remain locked for review; labels alone do not authorize edits.",
        "- Passing finite sweeps supports the tested gauge edits; it does not prove every value or simultaneous edit combination legal. Axial translation has no finite complete legal interval.",
        "- Increasing sample count tests more poses, not a stricter geometric tolerance. The injected link-length fault tests validator sensitivity separately from valid candidates.",
        "- Orientation uses clamped trace/acos and can show a numerical floor near 1e-8 rad; this is not a physical accuracy measurement.",
        "- Thresholds and report-specific rule labels remain provisional. No MATLAB execution/comparison or advisor acceptance is claimed.",
        "",
        f"All expected experiment outcomes met: **{result['expected_outcomes_met']}**.",
        "",
    ]
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source")
    parser.add_argument(
        "--output",
        required=True,
        help="New directory; existing output is never overwritten",
    )
    parser.add_argument(
        "--chain", type=int, help="Zero-based root-to-leaf path for a branched source"
    )
    args = parser.parse_args()
    output = Path(args.output)
    if output.exists():
        parser.error("Output directory already exists; choose a new directory")
    source = URDFInput.from_path(args.source)
    original_hash = source.sha256
    path = None
    if args.chain is not None:
        document = RobotDocument.load(args.source)
        if not 0 <= args.chain < len(document.paths):
            parser.error("Chain index is out of range")
        path = document.paths[args.chain]
        source = document.select(args.chain)
    result = run_studies(source)
    root = Path(__file__).resolve().parents[1]
    result["source"]["original_file_sha256"] = original_hash
    result["source"]["selected_path"] = path
    result["code_sha256"] = code_fingerprint(root)
    result["environment"] = {
        "python": platform.python_version(),
        "platform": platform.platform(),
        "versions": {
            name: version(name)
            for name in (
                "urdf2dt",
                "numpy",
                "scipy",
                "casadi",
                "defusedxml",
                "matplotlib",
            )
        },
        "utc": datetime.now(timezone.utc).isoformat(),
    }
    try:
        result["source"]["repository_commit"] = subprocess.check_output(
            ["git", "-C", str(Path(args.source).resolve().parent), "rev-parse", "HEAD"],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        result["source"]["repository_commit"] = None
    write_tables(result, output)
    plot_studies(result, output)
    (output / "summary.md").write_text(summary(result), encoding="utf-8")
    print(
        "Stage 14 results written. Expected outcomes met:",
        result["expected_outcomes_met"],
    )
    return 0 if result["expected_outcomes_met"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

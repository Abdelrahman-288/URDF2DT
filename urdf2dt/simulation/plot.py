"""Static scientific plots from checksummed simulation records."""

from pathlib import Path
import numpy as np
from urdf2dt.identification.data import read_record


def plot_study(directory: str | Path) -> Path:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    directory = Path(directory)
    report = read_record(directory/"report.json", "urdf2dt.simulation_study")
    runs = {mode: read_record(directory/f"{mode}-scale-1.json", "urdf2dt.simulation_run")
            for mode in ("pd", "pd_gravity", "computed_torque")}
    sample = runs["computed_torque"]["run"]
    n = len(sample["position"][0])
    fig, axes = plt.subplots(n, 2, figsize=(12, 2.2*n), squeeze=False, sharex=True)
    colors = {"pd": "#cc6633", "pd_gravity": "#228866", "computed_torque": "#3366cc"}
    for j in range(n):
        for mode, record in runs.items():
            run = record["run"]
            error = np.asarray(run["position"])-np.asarray([c["desired_position"] for c in run["commands"]])
            axes[j,0].plot(run["time"],error[:,j],label=mode,color=colors[mode])
            axes[j,1].step(run["time"],np.asarray([c["effort"] for c in run["commands"]])[:,j],
                           where="post",label=mode,color=colors[mode])
        axes[j,0].set_ylabel(f"Joint {j+1} error ({record['metrics']['position_units'][j]})")
        axes[j,1].set_ylabel(f"Effort ({record['metrics']['effort_units'][j]})")
        for ax in axes[j]:
            ax.axvline(2.,color="gray",linestyle=":",linewidth=1)
            ax.grid(alpha=.2)
    axes[0,0].legend(fontsize=8)
    axes[0,0].set_title("Position tracking error")
    axes[0,1].set_title("Applied effort (held at 100 Hz)")
    for ax in axes[-1]:
        ax.set_xlabel("Time (s); stationary reference after 2 s")
    fig.suptitle(f"{report['robot'].upper()} — nominal controller models, RK4 dt=2 ms")
    fig.tight_layout(rect=(0,0,1,.97))
    target = directory/"nominal-tracking.png"
    fig.savefig(target,dpi=150)
    plt.close(fig)
    return target

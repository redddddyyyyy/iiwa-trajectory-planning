"""Analyze a 7-DOF joint trajectory exported as a whitespace-delimited text file.

Usage:
  python scripts/analyze_trajectory.py data/trajectory_raw_4001x7.txt --dt 0.005 --outdir assets

Outputs:
  - joint_angles.png
  - joint_velocities.png
  - summary.json
"""

import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trajectory", type=str, help="Path to trajectory text file (Nx7).")
    ap.add_argument("--dt", type=float, default=0.005, help="Sample time in seconds.")
    ap.add_argument("--outdir", type=str, default="assets", help="Output directory.")
    args = ap.parse_args()

    traj_path = Path(args.trajectory)
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    q = np.loadtxt(traj_path)
    if q.ndim != 2 or q.shape[1] != 7:
        raise ValueError(f"Expected Nx7 trajectory, got shape {q.shape}")

    dt = float(args.dt)
    t = np.arange(q.shape[0]) * dt
    qd = np.vstack([np.diff(q, axis=0) / dt, np.zeros((1, 7))])

    speed = np.max(np.abs(qd), axis=1)
    spike_idx = int(np.argmax(speed))
    spike_t = float(t[spike_idx])

    # Joint angles
    plt.figure()
    for j in range(7):
        plt.plot(t, q[:, j], label=f"q{j+1}")
    plt.axvline(spike_t)
    plt.title("Joint Angles")
    plt.xlabel("Time [s]")
    plt.ylabel("rad")
    plt.legend(loc="best", ncol=2, fontsize=8)
    plt.grid(True)
    plt.savefig(outdir / "joint_angles.png", dpi=200, bbox_inches="tight")
    plt.close()

    # Joint velocities
    plt.figure()
    for j in range(7):
        plt.plot(t, qd[:, j], label=f"qd{j+1}")
    plt.axvline(spike_t)
    plt.title("Joint Velocities")
    plt.xlabel("Time [s]")
    plt.ylabel("rad/s")
    plt.legend(loc="best", ncol=2, fontsize=8)
    plt.grid(True)
    plt.savefig(outdir / "joint_velocities.png", dpi=200, bbox_inches="tight")
    plt.close()

    summary = {
        "samples": int(q.shape[0]),
        "dt_s": dt,
        "duration_s": float(t[-1]),
        "max_abs_joint_velocity_rad_s": [float(x) for x in np.max(np.abs(qd), axis=0)],
        "global_max_abs_joint_velocity_rad_s": float(np.max(np.abs(qd))),
        "spike_index": spike_idx,
        "spike_time_s": spike_t,
    }
    (outdir / "summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()

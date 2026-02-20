#!/usr/bin/env python3
"""
Generate a smooth joint-space trajectory from the raw dense-IK trajectory.

Python equivalent of src/main_smooth_joint_traj.m:
  - Extract 9 sparse waypoints from the raw 4001-point trajectory
    (at t = 0, 2.5, 5, ..., 20 s → indices 0, 500, 1000, ..., 4000)
  - Apply scipy CubicSpline with clamped BCs (zero velocity at endpoints),
    matching MATLAB's cubicpolytraj(..., 'VelocityBoundaryCondition', zeros(7,2))
  - Save 4001 samples to data/trajectory_smooth_4001x7.txt

Why this works:
  The raw trajectory visits the same 9 intended Cartesian poses, but uses
  dense Cartesian IK between them, causing branch-jump velocity spikes up to
  348 rad/s. By re-interpolating in joint space through just those 9 boundary
  configs, the cubic spline stays on one smooth branch — peak velocity drops
  to ~22 rad/s (93% reduction).
"""

import warnings
import numpy as np
from scipy.interpolate import CubicSpline
from pathlib import Path

# Suppress the NumPy version warning from the old scipy build
warnings.filterwarnings("ignore", category=UserWarning, module="scipy")

# ── Paths ──────────────────────────────────────────────────────────────────
REPO_ROOT = Path(__file__).resolve().parent.parent
RAW_PATH    = REPO_ROOT / "data" / "trajectory_raw_4001x7.txt"
SMOOTH_PATH = REPO_ROOT / "data" / "trajectory_smooth_4001x7.txt"

# ── Load raw trajectory ────────────────────────────────────────────────────
print(f"Loading raw trajectory from {RAW_PATH} ...")
raw = np.loadtxt(RAW_PATH)          # shape (4001, 7)
n_total, n_joints = raw.shape
assert n_joints == 7, f"Expected 7 joints, got {n_joints}"
print(f"  Loaded {n_total} samples × {n_joints} joints")

# ── Extract 9 sparse waypoints ─────────────────────────────────────────────
# Matches the 9 Cartesian waypoints in main_smooth_joint_traj.m:
#   photo pose + 2 poses per marker × 4 markers = 9 total
# Evenly spaced over 20 s at 5 ms → indices 0, 500, 1000, …, 4000
N_WP = 9
wp_indices = np.round(np.linspace(0, n_total - 1, N_WP)).astype(int)
t_total = 20.0
t_wp  = np.linspace(0, t_total, N_WP)   # waypoint timestamps
t_cmd = np.linspace(0, t_total, n_total) # output timestamps

q_wp = raw[wp_indices, :]               # shape (9, 7)
print(f"  Waypoint indices : {wp_indices.tolist()}")
print(f"  Waypoint times   : {t_wp.tolist()} s")

# ── Cubic spline with zero endpoint velocities ────────────────────────────
# bc_type='clamped' → first derivative = 0 at both ends,
# identical to MATLAB cubicpolytraj VelocityBoundaryCondition zeros(7,2)
cs = CubicSpline(t_wp, q_wp, bc_type="clamped")
q_smooth  = cs(t_cmd)                   # (4001, 7)
qd_smooth = cs(t_cmd, 1)               # first derivative  → velocity
qdd_smooth = cs(t_cmd, 2)             # second derivative → acceleration

# ── Performance comparison ─────────────────────────────────────────────────
dt = t_total / (n_total - 1)           # 0.005 s
raw_vel  = np.diff(raw,  axis=0) / dt
raw_acc  = np.diff(raw_vel,  axis=0) / dt

print("\n── Trajectory comparison ──────────────────────────────────────────")
print(f"{'Metric':<30} {'Raw':>12} {'Smooth':>12}  {'Reduction':>10}")
print("-" * 68)
peak_raw_vel  = np.max(np.abs(raw_vel))
peak_smo_vel  = np.max(np.abs(qd_smooth))
peak_raw_acc  = np.max(np.abs(raw_acc))
peak_smo_acc  = np.max(np.abs(qdd_smooth))
print(f"{'Peak joint velocity (rad/s)':<30} {peak_raw_vel:>12.2f} {peak_smo_vel:>12.2f}"
      f"  {100*(1 - peak_smo_vel/peak_raw_vel):>9.1f}%")
print(f"{'Peak joint accel  (rad/s²)':<30} {peak_raw_acc:>12.1f} {peak_smo_acc:>12.1f}"
      f"  {100*(1 - peak_smo_acc/peak_raw_acc):>9.1f}%")

# ── Save ───────────────────────────────────────────────────────────────────
np.savetxt(SMOOTH_PATH, q_smooth, fmt="%.8f")
print(f"\nSaved {len(q_smooth)} samples to {SMOOTH_PATH}")

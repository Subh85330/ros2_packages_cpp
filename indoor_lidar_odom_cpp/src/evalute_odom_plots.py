#!/usr/bin/env python3
import math

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def wrap_angle_rad(a):
    """Wrap angle to [-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def main():
    # Load CSVs
    est = pd.read_csv('odom_est.csv')
    gt = pd.read_csv('odom_sim.csv')

    # Convert to numpy for speed
    t_est = est['t'].to_numpy()
    x_est = est['x'].to_numpy()
    y_est = est['y'].to_numpy()
    yaw_est = est['yaw'].to_numpy()

    t_gt = gt['t'].to_numpy()
    x_gt = gt['x'].to_numpy()
    y_gt = gt['y'].to_numpy()
    yaw_gt = gt['yaw'].to_numpy()

    # Align trajectories by time: for each est t, find nearest gt t
    idx_gt_for_est = np.searchsorted(t_gt, t_est)
    idx_gt_for_est = np.clip(idx_gt_for_est, 0, len(t_gt) - 1)

    # Extract aligned gt samples
    x_gt_aligned = x_gt[idx_gt_for_est]
    y_gt_aligned = y_gt[idx_gt_for_est]
    yaw_gt_aligned = yaw_gt[idx_gt_for_est]
    t_aligned = t_est  # use estimated timestamps

    # Compute position error (per-sample ATE)
    dx = x_est - x_gt_aligned
    dy = y_est - y_gt_aligned
    pos_err = np.sqrt(dx * dx + dy * dy)  # meters

    # Compute yaw error (wrapped, in degrees)
    dyaw = np.array([wrap_angle_rad(e - g) for e, g in zip(yaw_est, yaw_gt_aligned)])
    yaw_err_deg = np.rad2deg(np.abs(dyaw))

    # Summary metrics
    ate_mean = float(np.mean(pos_err))
    ate_rmse = float(np.sqrt(np.mean(pos_err ** 2)))
    yaw_err_mean_deg = float(np.mean(yaw_err_deg))

    print(f"Mean ATE (m)     : {ate_mean:.3f}")
    print(f"RMSE ATE (m)     : {ate_rmse:.3f}")
    print(f"Mean yaw error(deg): {yaw_err_mean_deg:.3f}")

    # ---- Plot 1: trajectory (x–y) ----
    plt.figure()
    plt.plot(x_gt, y_gt, label='Ground Truth (odom_sim)')
    plt.plot(x_est, y_est, label='Estimated (odom_est)')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.title('Trajectory: estimated vs ground truth')
    plt.axis('equal')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('traj_est_vs_gt.png', dpi=150)

    # ---- Plot 2: error over time ----
    # You can choose either position error or yaw error; here I show both in separate figures.

    # 2a) Position error (ATE) over time
    plt.figure()
    plt.plot(t_aligned - t_aligned[0], pos_err)
    plt.xlabel('time [s]')
    plt.ylabel('position error [m]')
    plt.title('ATE over time')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('ate_over_time.png', dpi=150)

    # 2b) Yaw error over time (optional extra)
    plt.figure()
    plt.plot(t_aligned - t_aligned[0], yaw_err_deg)
    plt.xlabel('time [s]')
    plt.ylabel('yaw error [deg]')
    plt.title('Yaw error over time')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('yaw_error_over_time.png', dpi=150)

    plt.show()


if __name__ == '__main__':
    main()

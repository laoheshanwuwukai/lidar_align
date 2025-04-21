
import pandas as pd
import argparse
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R


def load_data(file_path):
    """加载数据，并计算欧拉角（单位：度）"""
    df = pd.read_csv(file_path, delim_whitespace=True, header=None,
                     names=['t', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])

    # 四元数 → 欧拉角（单位：度）
    rot = R.from_quat(df[['qx', 'qy', 'qz', 'qw']].to_numpy())
    euler_angles = rot.as_euler('xyz', degrees=True)

    df['roll'] = euler_angles[:, 0]
    df['pitch'] = euler_angles[:, 1]

    # 处理 yaw 跳变
    yaw_rad = np.radians(euler_angles[:, 2])
    yaw_unwrapped = np.unwrap(yaw_rad)
    df['yaw'] = np.degrees(yaw_unwrapped)

    return df


def plot_xyz_rpy(df1, df2, label1='File 1', label2='File 2'):
    """绘制 XYZ 和 RPY 的三个子图（竖排）"""

    time1 = df1['t'].to_numpy()
    time2 = df2['t'].to_numpy()

    # ========== Plot XYZ ==========
    fig_xyz, axes_xyz = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    xyz_labels = ['X', 'Y', 'Z']

    for i, axis in enumerate(['x', 'y', 'z']):
        axes_xyz[i].plot(time1, df1[axis].to_numpy(),
                         label=f'{label1} {axis.upper()}')
        axes_xyz[i].plot(time2, df2[axis].to_numpy(),
                         linestyle='--', label=f'{label2} {axis.upper()}')
        axes_xyz[i].set_ylabel(f'{xyz_labels[i]} (m)')
        axes_xyz[i].legend()
        axes_xyz[i].grid(True)

    axes_xyz[-1].set_xlabel('Time (s)')
    fig_xyz.suptitle('Position over Time')
    fig_xyz.tight_layout(rect=[0, 0.03, 1, 0.95])  # 留出标题空间

    # ========== Plot RPY ==========
    fig_rpy, axes_rpy = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    rpy_labels = ['Roll', 'Pitch', 'Yaw']

    for i, angle in enumerate(['roll', 'pitch', 'yaw']):
        axes_rpy[i].plot(time1, df1[angle].to_numpy(),
                         label=f'{label1} {rpy_labels[i]}')
        axes_rpy[i].plot(time2, df2[angle].to_numpy(),
                         linestyle='--', label=f'{label2} {rpy_labels[i]}')
        axes_rpy[i].set_ylabel(f'{rpy_labels[i]} (°)')
        axes_rpy[i].legend()
        axes_rpy[i].grid(True)

    axes_rpy[-1].set_xlabel('Time (s)')
    fig_rpy.suptitle('Euler Angles over Time (Unwrapped)')
    fig_rpy.tight_layout(rect=[0, 0.03, 1, 0.95])

    plt.show()


if __name__ == "__main__":
    args = argparse.ArgumentParser()
    args.add_argument("--ref", type=str, required=True)
    args.add_argument("--est", type=str, required=True)
    args = args.parse_args()

    ref_path = args.ref
    est_path = args.est

    df1 = load_data(ref_path)
    df2 = load_data(est_path)

    plot_xyz_rpy(df1, df2, "ref:lidar_mapping", "est:Sensor")

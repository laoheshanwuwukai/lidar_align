#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


def plot_two_txt_files_nearest_match(file_a, file_b):
    """
    对齐 file_b 到 file_a 的时间戳，按列画图（roll/pitch/yaw 转角度范围 -180~180）。
    """
    df_a = pd.read_csv(file_a, delim_whitespace=True, header=None)
    df_b = pd.read_csv(file_b, delim_whitespace=True, header=None)

    if df_a.shape[1] != 7 or df_b.shape[1] != 7:
        raise ValueError("两个文件必须都包含7列（时间戳 + 6个数据列）")

    # 将 roll/pitch/yaw 从弧度转为角度，并限制在 [-180, 180]
    for i in range(4, 7):
        df_a.iloc[:, i] = (np.degrees(df_a.iloc[:, i]) + 180) % 360 - 180
        df_b.iloc[:, i] = (np.degrees(df_b.iloc[:, i]) + 180) % 360 - 180

    # 以 df_a 的时间戳为基准
    t_a = df_a.iloc[:, 0].to_numpy()
    t_b = df_b.iloc[:, 0].to_numpy()

    # 对 df_b 做索引匹配：找到与 t_a 最接近的 t_b 行
    # shape: (len(t_a),)
    idx_b_matched = np.abs(t_b[:, None] - t_a).argmin(axis=0)
    df_b_aligned = df_b.iloc[idx_b_matched, :].reset_index(drop=True)

    # 画图
    titles = ["t", "x", "y", "z", "roll", "pitch", "yaw"]

    for i in range(1, 7):
        name = titles[i]
        plt.figure()
        plt.plot(t_a, df_a.iloc[:, i].to_numpy(),
                 label=f'ground truth - {name}')
        plt.plot(t_a, df_b_aligned.iloc[:, i].to_numpy(
        ), label=f'Ins - {name}', linestyle='--')
        plt.xlabel('Timestamp (s)')
        plt.ylabel(f'{name} {"(°)" if i >= 4 else ""}')
        plt.title(f'{name} over Time (Aligned by Nearest Timestamp)')
        plt.legend()
        plt.grid(True)
        plt.tight_layout()

    plt.show()


if __name__ == "__main__":
    mapping_result = "/home/udeer/data/InsGj/0408/mapping_result/mapping_result_in_lidar_rpy"
    gongji_result = "/home/udeer/data/InsGj/0408/_udeer_drivers_gnss_gjins_ins_raw"
    plot_two_txt_files_nearest_match(mapping_result, gongji_result)

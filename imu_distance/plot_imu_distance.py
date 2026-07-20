import os
import sys
import csv

def try_imports():
    try:
        import pandas as pd
    except Exception:
        pd = None
    try:
        import matplotlib.pyplot as plt
    except Exception:
        plt = None
    try:
        import numpy as np
    except Exception:
        np = None
    return pd, plt, np

def read_csv_fallback(path):
    times = []
    cols = []
    with open(path, newline='', encoding='utf-8') as f:
        r = csv.reader(f)
        header = next(r, None)
        for row in r:
            if not row:
                continue
            if len(times) == 0:
                cols = [[] for _ in range(len(row)-1)]
            try:
                t = float(row[0])
            except Exception:
                continue
            times.append(t)
            for i in range(1, len(row)):
                try:
                    cols[i-1].append(float(row[i]))
                except Exception:
                    cols[i-1].append(float('nan'))
    return times, cols, header

def main():
    base = os.path.dirname(__file__)
    acc_path = os.path.join(base, 'ACC.csv')
    gyro_path = os.path.join(base, 'GYRO.csv')
    dist_path = os.path.join(base, 'Distance.csv')
    out_dir = base

    pd, plt, np = try_imports()
    if plt is None:
        print('matplotlib is required to generate plots. Install it with: pip install matplotlib')
        return

    if pd is not None:
        try:
            acc = pd.read_csv(acc_path) if os.path.exists(acc_path) else None
        except Exception as e:
            print('pandas failed to read ACC.csv:', e)
            acc = None
        try:
            gyro = pd.read_csv(gyro_path) if os.path.exists(gyro_path) else None
        except Exception as e:
            print('pandas failed to read GYRO.csv:', e)
            gyro = None
        try:
            dist = pd.read_csv(dist_path) if os.path.exists(dist_path) else None
        except Exception as e:
            print('pandas failed to read Distance.csv:', e)
            dist = None
    else:
        acc = gyro = dist = None

    if acc is None and os.path.exists(acc_path):
        times, cols, header = read_csv_fallback(acc_path)
        acc = {'Time': times}
        for i, name in enumerate(header[1:]):
            acc[name] = cols[i]

    if gyro is None and os.path.exists(gyro_path):
        times_g, cols_g, header_g = read_csv_fallback(gyro_path)
        gyro = {'Time': times_g}
        for i, name in enumerate(header_g[1:]):
            gyro[name] = cols_g[i]

    if dist is None and os.path.exists(dist_path):
        times_d, cols_d, header_d = read_csv_fallback(dist_path)
        dist = {'Time': times_d, header_d[1]: cols_d[0]}

    if acc:
        fig, ax = plt.subplots()
        t = acc['Time']
        labels = [k for k in acc.keys() if k != 'Time']
        for lbl in labels:
            ax.plot(t, acc[lbl], label=lbl)
        ax.set_title('Accelerometer')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Acceleration')
        ax.legend()
        fig.tight_layout()
        out = os.path.join(out_dir, 'ACC_plot.png')
        fig.savefig(out)
        plt.close(fig)
        print('Saved', out)

    if gyro:
        fig, ax = plt.subplots()
        t = gyro['Time']
        labels = [k for k in gyro.keys() if k != 'Time']
        for lbl in labels:
            ax.plot(t, gyro[lbl], label=lbl)
        ax.set_title('Gyroscope')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angular velocity')
        ax.legend()
        fig.tight_layout()
        out = os.path.join(out_dir, 'GYRO_plot.png')
        fig.savefig(out)
        plt.close(fig)
        print('Saved', out)

    if dist:
        fig, ax = plt.subplots()
        t = dist['Time']
        dvals = None
        for k in dist.keys():
            if k != 'Time':
                dvals = dist[k]
                break
        ax.plot(t, dvals, label='Distance')
        ax.set_title('Distance Sensor')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Distance (mm)')
        ax.legend()
        fig.tight_layout()
        out = os.path.join(out_dir, 'Distance_plot.png')
        fig.savefig(out)
        plt.close(fig)
        print('Saved', out)

if __name__ == '__main__':
    main()

import os
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
    return pd, plt

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

    pd, plt = try_imports()
    if plt is None:
        print('matplotlib is required. Install: pip install matplotlib')
        return

    acc = gyro = dist = None
    if pd is not None:
        try:
            if os.path.exists(acc_path):
                acc = pd.read_csv(acc_path)
        except Exception:
            acc = None
        try:
            if os.path.exists(gyro_path):
                gyro = pd.read_csv(gyro_path)
        except Exception:
            gyro = None
        try:
            if os.path.exists(dist_path):
                dist = pd.read_csv(dist_path)
        except Exception:
            dist = None

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

    out_dir = base

    if acc:
        fig, ax = plt.subplots()
        t = acc['Time']
        for k in [k for k in acc.keys() if k!='Time']:
            ax.plot(t, acc[k], label=k)
        ax.set_title('Accelerometer (scenerio_1)')
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
        for k in [k for k in gyro.keys() if k!='Time']:
            ax.plot(t, gyro[k], label=k)
        ax.set_title('Gyroscope (scenerio_1)')
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
            if k!='Time':
                dvals = dist[k]
                break
        ax.plot(t, dvals, label='Distance')
        ax.set_title('Distance (scenerio_1)')
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

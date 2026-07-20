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

def has_data(obj):
    if obj is None:
        return False
    if hasattr(obj, 'empty'):
        try:
            return not obj.empty
        except Exception:
            pass
    if isinstance(obj, dict):
        return ('Time' in obj) and (len(obj['Time']) > 0)
    return True

def main():
    base = os.path.dirname(__file__)
    acc_path = os.path.join(base, 'ACC.csv')
    gyro_path = os.path.join(base, 'GYRO.csv')
    dist_path = os.path.join(base, 'Distance.csv')

    pd, plt = try_imports()
    if plt is None:
        print('matplotlib required: pip install matplotlib')
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

    if has_data(acc):
        fig, ax = plt.subplots()
        if hasattr(acc, 'columns'):
            t = acc['Time']
            cols_iter = [c for c in acc.columns if c != 'Time']
            for k in cols_iter:
                ax.plot(t, acc[k], label=k)
        else:
            t = acc['Time']
            for k in [k for k in acc.keys() if k != 'Time']:
                ax.plot(t, acc[k], label=k)
        ax.set_title('ACC (scenerio_4)')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Acceleration')
        ax.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(base, 'ACC_plot.png'))
        plt.close(fig)
        print('Saved ACC_plot.png')

    if has_data(gyro):
        fig, ax = plt.subplots()
        if hasattr(gyro, 'columns'):
            t = gyro['Time']
            cols_iter = [c for c in gyro.columns if c != 'Time']
            for k in cols_iter:
                ax.plot(t, gyro[k], label=k)
        else:
            t = gyro['Time']
            for k in [k for k in gyro.keys() if k != 'Time']:
                ax.plot(t, gyro[k], label=k)
        ax.set_title('GYRO (scenerio_4)')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angular velocity')
        ax.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(base, 'GYRO_plot.png'))
        plt.close(fig)
        print('Saved GYRO_plot.png')

    if has_data(dist):
        fig, ax = plt.subplots()
        if hasattr(dist, 'columns'):
            t = dist['Time']
            dcol = [c for c in dist.columns if c != 'Time'][0]
            dvals = dist[dcol]
        else:
            t = dist['Time']
            dvals = None
            for k in dist.keys():
                if k != 'Time':
                    dvals = dist[k]
                    break
        ax.plot(t, dvals, label='Distance')
        ax.set_title('Distance (scenerio_4)')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Distance (mm)')
        ax.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(base, 'Distance_plot.png'))
        plt.close(fig)
        print('Saved Distance_plot.png')

if __name__ == '__main__':
    main()

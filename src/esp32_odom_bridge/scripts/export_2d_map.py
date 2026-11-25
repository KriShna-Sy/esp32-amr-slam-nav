#!/usr/bin/env python3
"""
export_2d_map.py - pull the optimized 2D occupancy grid out of RTAB-Map databases
and render them side by side (no ROS needed, reads the .db directly).

  python3 export_2d_map.py <map1.db> [<map2.db> ...] [-o out.png]

RTAB-Map stores the last optimized 2D grid in the Admin table (opt_map, compressed
cv::Mat: zlib stream + 3 int32 [rows, cols, type] appended). Landmarks are overlaid
if a landmarks.yaml sits next to the db.
"""
import os
import sys
import zlib
import struct
import sqlite3
import numpy as np


def unpack_mat(blob):
    """RTAB-Map compressData2 -> numpy. Tries both trailer layouts."""
    if blob is None or len(blob) < 13:
        return None
    raw = None
    for trailer in (12,):
        try:
            raw = zlib.decompress(bytes(blob[:-trailer]))
            tail = struct.unpack('<3i', bytes(blob[-trailer:]))
        except Exception:
            continue
        for rows, cols, typ in (tail, (tail[1], tail[0], tail[2])):
            depth = typ & 7
            ch = (typ >> 3) + 1
            size = {0: 1, 1: 1, 2: 2, 3: 2, 4: 4, 5: 4, 6: 8}.get(depth, 1)
            if rows > 0 and cols > 0 and rows * cols * ch * size == len(raw):
                dt = {0: np.uint8, 1: np.int8, 2: np.uint16, 3: np.int16,
                      4: np.int32, 5: np.float32, 6: np.float64}[depth]
                a = np.frombuffer(raw, dtype=dt)
                return a.reshape(rows, cols) if ch == 1 else a.reshape(rows, cols, ch)
    return None


def load_grid(db):
    c = sqlite3.connect(f'file:{db}?mode=ro', uri=True).cursor()
    cols = [r[1] for r in c.execute('PRAGMA table_info(Admin)')]
    if 'opt_map' not in cols:
        raise RuntimeError('this RTAB-Map version has no Admin.opt_map')
    row = c.execute('SELECT opt_map, opt_map_x_min, opt_map_y_min, opt_map_resolution FROM Admin').fetchone()
    if row is None or row[0] is None:
        raise RuntimeError('no saved 2D map in this db (open it once in rtabmap-databaseViewer, or map was not closed cleanly)')
    grid = unpack_mat(row[0])
    if grid is None:
        raise RuntimeError('could not decode opt_map')
    nodes = c.execute('SELECT count(*) FROM Node').fetchone()[0]
    sessions = c.execute('SELECT count(DISTINCT map_id) FROM Node').fetchone()[0]
    return grid.astype(np.int16), float(row[1]), float(row[2]), float(row[3]), nodes, sessions


def load_landmarks(db):
    p = os.path.join(os.path.dirname(os.path.abspath(db)), 'landmarks.yaml')
    try:
        import yaml
        d = yaml.safe_load(open(p)) or {}
        return {k: (v['x'], v['y'], v.get('yaw_deg', 0.0)) for k, v in d.items()}
    except Exception:
        return {}


def stats(g):
    occ = (g >= 50).sum()
    free = ((g >= 0) & (g < 50)).sum()
    return occ, free


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('-o')]
    out = os.path.expanduser('~/mapping_ws/logs/map2d_compare.png')
    if '-o' in sys.argv:
        out = sys.argv[sys.argv.index('-o') + 1]
        args = [a for a in args if a != out]
    if not args:
        print(__doc__)
        sys.exit(1)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, len(args), figsize=(9 * len(args), 9), squeeze=False)
    for ax, db in zip(axes[0], args):
        name = os.path.basename(os.path.dirname(os.path.abspath(db))) + '/' + os.path.basename(db)
        try:
            g, xmin, ymin, res, nodes, sess = load_grid(db)
        except Exception as e:
            ax.set_title(f'{name}\n{e}', fontsize=9)
            print(f'[ERROR] {db}: {e}')
            continue
        h, w = g.shape
        img = np.full((h, w, 3), 0.75)            # unknown = grey
        img[(g >= 0) & (g < 50)] = (1, 1, 1)      # free = white
        img[g >= 50] = (0, 0, 0)                  # occupied = black
        ext = [xmin, xmin + w * res, ymin, ymin + h * res]
        ax.imshow(img, origin='lower', extent=ext, interpolation='nearest')
        for k, (x, y, yaw) in load_landmarks(db).items():
            ax.plot(x, y, 'ro', ms=6)
            ax.arrow(x, y, 0.3 * np.cos(np.radians(yaw)), 0.3 * np.sin(np.radians(yaw)),
                     color='r', head_width=0.08)
            ax.text(x + 0.1, y + 0.1, k, color='r', fontsize=9)
        ax.plot(0, 0, 'b+', ms=12, mew=2)
        occ, free = stats(g)
        ax.set_title(f'{name}\n{nodes} nodes, {sess} session(s), {w*res:.1f} x {h*res:.1f} m @ {res*100:.0f} cm\n'
                     f'occupied {occ*res*res:.1f} m2, free {free*res*res:.1f} m2', fontsize=10)
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.grid(alpha=0.3)
        ax.set_aspect('equal')
        print(f'[OK] {db}: {w}x{h} @ {res:.3f} m, origin ({xmin:.2f}, {ymin:.2f}), {nodes} nodes, {sess} sessions')
    plt.tight_layout()
    os.makedirs(os.path.dirname(out), exist_ok=True)
    plt.savefig(out, dpi=90)
    print(f'[OK] picture: {out}')


if __name__ == '__main__':
    main()

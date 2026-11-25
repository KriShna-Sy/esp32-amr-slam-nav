#!/usr/bin/env bash
# Freeze the last mapping run as THE navigation map (fresh data folder only).
#   ros2 run esp32_odom_bridge save_nav_map.sh <name>              (copies ~/.ros/rtabmap.db)
#   ros2 run esp32_odom_bridge save_nav_map.sh <name> other.db     (copies a given db)
# The map that was active before is ARCHIVED (never deleted) to nav_data/archive/<its name>/
# Switch between saved maps:  ros2 run esp32_odom_bridge nav_map_switch.sh [name|list]
# Result in ~/mapping_ws/nav_data/:
#   nav_map.db        <- localization loads ONLY this
#   nav_map_cloud.ply (3D check) + nav_map_top.png (top view)
#   landmarks.yaml    <- emptied: landmarks of a previous map are invalid
#   camera.yaml       <- camera mount used for THIS map (localization reads it)
#   PROFILE           <- name of the active map
set -e
NAME="${1:-map_$(date +%Y%m%d_%H%M)}"
SRC="${2:-$HOME/.ros/rtabmap.db}"
D="$HOME/mapping_ws/nav_data"
A="$D/archive"
[ -f "$SRC" ] || { echo "[ERROR] $SRC not found"; exit 1; }
[ -e "$A/$NAME" ] && { echo "[ERROR] archive/$NAME already exists - pick another name"; exit 1; }
mkdir -p "$A"
python3 - "$SRC" <<'PY'
import sqlite3, sys, math, numpy as np
c = sqlite3.connect(sys.argv[1]).cursor()
n = c.execute('SELECT count(*) FROM Node').fetchone()[0]
lc = c.execute('SELECT count(*) FROM Link WHERE type=1').fetchone()[0]
rows = [(mid, np.frombuffer(p, '<f4').reshape(3, 4)) for (mid, p) in
        c.execute('SELECT map_id, pose FROM Node ORDER BY id') if p]
sessions = sorted(set(r[0] for r in rows))
L = 0.0
for s in sessions:
    Q = np.array([m[:2, 3] for mid, m in rows if mid == s])
    if len(Q) > 1:
        L += float(np.sum(np.linalg.norm(np.diff(Q, axis=0), axis=1)))
P = np.array([m[:2, 3] for _, m in rows])
print(f"[INFO] {n} nodes | {lc} loop-closure links | path {L:.1f} m | "
      f"extent {np.ptp(P[:,0]):.1f} x {np.ptp(P[:,1]):.1f} m | sessions {len(sessions)}")
if len(sessions) > 1:
    print(f"[WARN] {len(sessions)} mapping sessions in one db (mapping was started without a fresh db).")
    print("       Mixing sessions (e.g. different camera mounts) gives doubled/smeared walls.")
    print("       Remap with: ros2 launch esp32_odom_bridge mapping.launch.py   (fresh db is now the default)")
if L < 5.0:
    print("[WARN] path < 5 m: this is not a full room map.")
per_m = lc / max(L, 1e-6)
print(f"[QA] loop closures per metre: {per_m:.2f}  (target >= 1.0)")
print("[QA] " + ("PASS" if per_m >= 1.0 and len(sessions) == 1 else
                 "CHECK: few loop closures or several sessions - drive the weak areas again before saving landmarks"))
PY
# Pristine, read-only copy of the raw mapping data. Localization writes to nav_map.db
# (e.g. last pose), so THIS file is the master for any future offline rebuild:
#   bash ~/mapping_ws/src/esp32_odom_bridge/scripts/reprocess_nav_map.sh <new_name> [--Param value ...]
mkdir -p "$D/raw"
if [ ! -e "$D/raw/${NAME}_raw.db" ] && [[ "$SRC" != *reprocess_* ]]; then   # only real mapping runs are masters
  cp "$SRC" "$D/raw/${NAME}_raw.db" && chmod 444 "$D/raw/${NAME}_raw.db"
  echo "[OK] raw master copy (read-only): $D/raw/${NAME}_raw.db"
fi
if [ -f "$D/nav_map.db" ]; then
  OLD="$(cat "$D/PROFILE" 2>/dev/null || echo "unnamed_$(date +%Y%m%d_%H%M%S)")"
  [ -e "$A/$OLD" ] && OLD="${OLD}_$(date +%H%M%S)"
  mkdir -p "$A/$OLD"
  for f in nav_map.db landmarks.yaml keepout.yaml camera.yaml nav_map_cloud.ply nav_map_top.png; do
    [ -e "$D/$f" ] && mv -n "$D/$f" "$A/$OLD/$f"
  done
  echo "[INFO] previous map archived (not deleted): $A/$OLD"
fi
cp "$SRC" "$D/nav_map.db"
: > "$D/landmarks.yaml"
echo "$NAME" > "$D/PROFILE"
# camera mount used while mapping = current mapping.launch.py defaults
python3 - "$HOME/mapping_ws/src/esp32_odom_bridge/launch/mapping.launch.py" "$D/camera.yaml" <<'PY'
import re, sys
src = open(sys.argv[1]).read()
out = []
for k in ['cam_x', 'cam_y', 'cam_z', 'cam_roll', 'cam_pitch', 'cam_yaw']:
    m = re.search(r"DeclareLaunchArgument\('%s',\s*default_value='([^']+)'" % k, src)
    if m:
        out.append(f"{k}: {m.group(1)}")
open(sys.argv[2], 'w').write("\n".join(out) + "\n")
print("[OK] camera mount for this map: " + ", ".join(out))
PY
echo "[OK] active map name: $NAME"
echo "[OK] $D/nav_map.db   (landmarks.yaml is empty - save new ones)"
if command -v rtabmap-export >/dev/null; then
  rtabmap-export --cloud --voxel 0.03 --output_dir "$D" --output nav_map "$D/nav_map.db" >/dev/null 2>&1 || true
  if [ -f "$D/nav_map_cloud.ply" ]; then
    echo "[OK] $D/nav_map_cloud.ply"
    python3 - "$D/nav_map_cloud.ply" "$D/nav_map_top.png" <<'PY' || true
import sys, numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
d = open(sys.argv[1], 'rb').read()
h = d[:d.index(b'end_header\n')].decode(errors='ignore').splitlines()
n = int([l for l in h if l.startswith('element vertex')][0].split()[-1])
props, cur = [], None
for l in h:
    if l.startswith('element'): cur = l.split()[1]
    elif l.startswith('property') and cur == 'vertex': props.append(l.split())
tmap = {'float': '<f4', 'uchar': 'u1', 'double': '<f8', 'int': '<i4'}
dt = np.dtype([(p[2], tmap[p[1]]) for p in props if p[1] in tmap])
a = np.frombuffer(d[d.index(b'end_header\n') + 11:], dtype=dt, count=n)
m = (a['z'] > 0.1) & (a['z'] < 1.6)
plt.figure(figsize=(9, 9)); plt.scatter(a['x'][m], a['y'][m], s=0.3, c=a['z'][m])
plt.gca().set_aspect('equal'); plt.grid(); plt.title('obstacles 0.1-1.6 m (walls must be thin & straight)')
plt.savefig(sys.argv[2], dpi=80, bbox_inches='tight'); print('[OK] ' + sys.argv[2])
PY
  fi
fi
echo "[NEXT] ros2 launch esp32_odom_bridge autonomous_bringup.launch.py"
echo "       ros2 run esp32_odom_bridge landmark_manager save <name>"

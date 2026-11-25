#!/usr/bin/env bash
# Rebuild the navigation map OFFLINE from the raw mapping data - no driving needed.
# Use when you want different SLAM / grid / filter settings on the same recorded run.
#
#   bash ~/mapping_ws/src/esp32_odom_bridge/scripts/reprocess_nav_map.sh <new_name> [--Param value ...] [--from <raw.db>]
#   e.g.  ... reprocess_nav_map.sh cam080_v4b --Grid/RangeMax 2.0 --Vis/MinInliers 15
#
# Default input: the newest nav_data/raw/*_raw.db. Output becomes the active map via
# save_nav_map.sh (old map archived, landmarks.yaml emptied -> re-check / re-save landmarks).
set -e
NAME="$1"; shift || true
[ -n "$NAME" ] || { echo "usage: reprocess_nav_map.sh <new_name> [--Param value ...] [--from raw.db]"; exit 1; }
D="$HOME/mapping_ws/nav_data"
SRC=""
ARGS=()
while [ $# -gt 0 ]; do
  if [ "$1" = "--from" ]; then SRC="$2"; shift 2; else ARGS+=("$1"); shift; fi
done
[ -n "$SRC" ] || SRC="$(ls -t "$D"/raw/*_raw.db 2>/dev/null | head -1)"
[ -f "$SRC" ] || { echo "[ERROR] no raw map found in $D/raw/"; exit 1; }
command -v rtabmap-reprocess >/dev/null || { echo "[ERROR] rtabmap-reprocess not found (sudo apt install ros-jazzy-rtabmap)"; exit 1; }
OUT="$HOME/.ros/reprocess_${NAME}.db"
rm -f "$OUT"
echo "[INFO] input : $SRC"
echo "[INFO] params: ${ARGS[*]:-(as recorded)}"
# Same settings as mapping.launch.py, then your overrides (later values win)
rtabmap-reprocess \
  --Mem/RehearsalSimilarity 1.0 --Mem/BinDataKept true \
  --Reg/Force3DoF true --Reg/Strategy 0 --Vis/EstimationType 1 \
  --Vis/MinInliers 12 --Vis/PnPReprojError 4.5 --RGBD/OptimizeMaxError 3.0 \
  --RGBD/ProximityBySpace true --RGBD/NeighborLinkRefining true \
  --Grid/FromDepth true --Grid/CellSize 0.03 --Grid/RangeMin 0.35 --Grid/RangeMax 2.5 \
  --Grid/MaxGroundHeight 0.08 --Grid/MaxObstacleHeight 1.60 --Grid/NormalsSegmentation true \
  --Grid/NoiseFilteringRadius 0.05 --Grid/NoiseFilteringMinNeighbors 3 \
  "${ARGS[@]}" "$SRC" "$OUT"
echo "[INFO] reprocessed -> $OUT ; installing as active map '$NAME'"
bash "$(dirname "$0")/save_nav_map.sh" "$NAME" "$OUT"

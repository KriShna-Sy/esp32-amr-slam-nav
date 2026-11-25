#!/usr/bin/env bash
# Switch the active navigation map (nothing is ever deleted).
#   ros2 run esp32_odom_bridge nav_map_switch.sh list
#   ros2 run esp32_odom_bridge nav_map_switch.sh <name>
# Active map lives in ~/mapping_ws/nav_data/, others in nav_data/archive/<name>/.
# Each map keeps its own landmarks.yaml and camera.yaml (camera mount it was made with).
# Stop autonomous_bringup before switching.
set -e
D="$HOME/mapping_ws/nav_data"
A="$D/archive"
mkdir -p "$A"
ACTIVE="$(cat "$D/PROFILE" 2>/dev/null || echo unnamed)"
if [ -z "$1" ] || [ "$1" = "list" ]; then
  echo "active : $ACTIVE  $( [ -f "$D/camera.yaml" ] && tr '\n' ' ' < "$D/camera.yaml")"
  for d in "$A"/*/; do
    [ -d "$d" ] || continue
    n="$(basename "$d")"
    echo "archive: $n  $( [ -f "$d/camera.yaml" ] && tr '\n' ' ' < "$d/camera.yaml")"
  done
  exit 0
fi
NEW="$1"
[ -f "$A/$NEW/nav_map.db" ] || { echo "[ERROR] $A/$NEW/nav_map.db not found (try: list)"; exit 1; }
[ "$NEW" = "$ACTIVE" ] && { echo "[INFO] $NEW is already active"; exit 0; }
if pgrep -f "rtabmap" >/dev/null; then
  echo "[ERROR] rtabmap is running - stop autonomous_bringup / mapping first"; exit 1
fi
T="$A/$ACTIVE"; [ -e "$T" ] && T="$A/${ACTIVE}_$(date +%H%M%S)"
mkdir -p "$T"
for f in nav_map.db landmarks.yaml keepout.yaml camera.yaml nav_map_cloud.ply nav_map_top.png; do
  [ -e "$D/$f" ] && mv -n "$D/$f" "$T/$f"
done
for f in nav_map.db landmarks.yaml keepout.yaml camera.yaml nav_map_cloud.ply nav_map_top.png; do
  [ -e "$A/$NEW/$f" ] && mv -n "$A/$NEW/$f" "$D/$f"
done
rmdir "$A/$NEW" 2>/dev/null || true
[ -f "$D/landmarks.yaml" ] || : > "$D/landmarks.yaml"
echo "$NEW" > "$D/PROFILE"
echo "[OK] active map: $NEW   (previous '$ACTIVE' -> $T)"
[ -f "$D/camera.yaml" ] && echo "[OK] localization will use camera mount: $(tr '\n' ' ' < "$D/camera.yaml")"
echo "[!] the PHYSICAL camera must be mounted the same way as when this map was made."

#!/usr/bin/env bash
# Compile + upload amr_base_pid straight from the workspace file (no Arduino IDE buffer).
#   ros2 run esp32_odom_bridge flash_fw.sh [/dev/ttyUSB0]
# Close the Arduino IDE serial monitor / miniterm / ROS launch first.
set -e
PORT="${1:-/dev/ttyUSB0}"
# Master firmware. Do NOT open this folder in the Arduino IDE (its auto-save overwrites it).
SKETCH="$HOME/mapping_ws/src/esp32_odom_bridge/firmware/amr_base"
export PATH="$HOME/.local/bin:$PATH"

if ! command -v arduino-cli >/dev/null; then
  echo "[INFO] installing arduino-cli to ~/.local/bin ..."
  mkdir -p "$HOME/.local/bin"
  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR="$HOME/.local/bin" sh
fi
# reuse the ESP32 core the Arduino IDE already installed (~/.arduino15)
if ! arduino-cli core list | grep -q '^esp32:esp32'; then
  arduino-cli config init --overwrite >/dev/null 2>&1 || true
  arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
  arduino-cli core update-index
  arduino-cli core install esp32:esp32
fi

if pgrep -f -i "arduino-ide|arduino ide" >/dev/null; then
  echo "[ERROR] Arduino IDE is running - it auto-saves its OLD copy over this file. Close it, then rerun."
  exit 1
fi
echo "[INFO] sketch: $SKETCH/amr_base.ino"
grep -n 'READY - amr_base_pid' "$SKETCH/amr_base.ino"
arduino-cli compile --fqbn esp32:esp32:esp32 --upload -p "$PORT" "$SKETCH"
echo "[OK] flashed. Check the version with: ros2 run esp32_odom_bridge fw_test"

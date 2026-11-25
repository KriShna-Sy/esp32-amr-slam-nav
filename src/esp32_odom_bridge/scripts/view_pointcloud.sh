#!/bin/bash
# ==============================================================================
# Helper Script: View Robot 3D Point Clouds
# Usage:
#   ./view_pointcloud.sh              (Opens PCL Viewer by default)
#   ./view_pointcloud.sh meshlab      (Opens in MeshLab)
#   ./view_pointcloud.sh rtabmap      (Opens in RTAB-Map Database Viewer)
#   ./view_pointcloud.sh ros2         (Publishes to ROS 2 topic /cloud_pcd)
# ==============================================================================

MAPS_DIR="$HOME/mapping_ws/maps"

# Determine target file
TARGET="$1"
if [ "$TARGET" = "rtabmap" ] || [ "$TARGET" = "db" ]; then
    DB_FILE="${2:-$HOME/.ros/rtabmap.db}"
    echo "[LAUNCH] Opening RTAB-Map Database Viewer with $DB_FILE..."
    echo "  Tip: Press Ctrl+3 or Edit -> 'View 3D map' to display the cloud!"
    rtabmap-databaseViewer "$DB_FILE" &
    exit 0
fi

if [ "$TARGET" = "meshlab" ]; then
    PLY_FILE="${2:-$MAPS_DIR/calibrated_map_cloud_cloud.ply}"
    echo "[LAUNCH] Opening MeshLab with $PLY_FILE..."
    meshlab "$PLY_FILE" &
    exit 0
fi

# Find newest PCD or PLY, auto-exporting fresh rtabmap.db if newer
DB_FILE="$HOME/.ros/rtabmap.db"

if [ -n "$TARGET" ] && [ -f "$TARGET" ]; then
    INPUT_FILE="$TARGET"
else
    # Check if active rtabmap database is newer than the newest cloud file
    NEWEST_CLOUD=$(ls -t "$MAPS_DIR"/*.ply "$MAPS_DIR"/*.pcd 2>/dev/null | head -n 1)
    if [ -f "$DB_FILE" ]; then
        if [ -z "$NEWEST_CLOUD" ] || [ "$DB_FILE" -nt "$NEWEST_CLOUD" ]; then
            echo "[INFO] Detected new mapping session in ~/.ros/rtabmap.db! Exporting fresh 3D point cloud..."
            TIMESTAMP=$(date +%Y%m%d_%H%M%S)
            EXPORT_NAME="session_${TIMESTAMP}"
            rtabmap-export --cloud --voxel 0.02 --output_dir "$MAPS_DIR" --output "$EXPORT_NAME" "$DB_FILE" >/dev/null 2>&1
            if [ -f "$MAPS_DIR/${EXPORT_NAME}_cloud.ply" ]; then
                echo "[INFO] Converting fresh cloud to PCD format..."
                pcl_ply2pcd "$MAPS_DIR/${EXPORT_NAME}_cloud.ply" "$MAPS_DIR/${EXPORT_NAME}.pcd" >/dev/null 2>&1
                INPUT_FILE="$MAPS_DIR/${EXPORT_NAME}.pcd"
            fi
        fi
    fi

    if [ -z "$INPUT_FILE" ]; then
        INPUT_FILE=$(ls -t "$MAPS_DIR"/*.ply "$MAPS_DIR"/*.pcd 2>/dev/null | head -n 1)
    fi
fi

if [ -z "$INPUT_FILE" ]; then
    echo "[ERROR] No point cloud files found in $MAPS_DIR"
    exit 1
fi

# If input is a PLY file, convert to PCD for pcl_viewer
if [[ "$INPUT_FILE" == *.ply ]]; then
    PCD_FILE="${INPUT_FILE%.ply}.pcd"
    if [ ! -f "$PCD_FILE" ] || [ "$INPUT_FILE" -nt "$PCD_FILE" ]; then
        echo "[INFO] Converting $(basename "$INPUT_FILE") to PCD..."
        pcl_ply2pcd "$INPUT_FILE" "$PCD_FILE" >/dev/null 2>&1
    fi
else
    PCD_FILE="$INPUT_FILE"
fi

echo "[LAUNCH] Opening PCL Viewer with $PCD_FILE..."
echo "  Controls: [c] Toggle Color Mode | [+] [-] Change Point Size | [r] Reset View | [q] Quit"
pcl_viewer "$PCD_FILE"

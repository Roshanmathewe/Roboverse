#!/usr/bin/env bash
set -Ee -o pipefail
cd "$HOME/warehouse_robot_ros2"

source /opt/ros/jazzy/setup.bash
source "$HOME/warehouse_robot_ros2/install/setup.bash"

if ! python3 -c "import cv2, numpy" >/dev/null 2>&1; then
    echo "OpenCV or NumPy is missing. Install with: sudo apt install python3-opencv python3-numpy"
    exit 1
fi

if command -v fuser >/dev/null 2>&1 && fuser -n tcp 8080 >/dev/null 2>&1; then
    echo "Port 8080 is already in use. Stop the old dashboard and retry."
    exit 1
fi

SIM_PID=""
DASH_PID=""
cleanup() {
    echo "Stopping RoboVerse..."
    [[ -z "$DASH_PID" ]] || kill -INT "$DASH_PID" 2>/dev/null || true
    [[ -z "$SIM_PID" ]] || kill -INT "$SIM_PID" 2>/dev/null || true
    [[ -z "$DASH_PID" ]] || wait "$DASH_PID" 2>/dev/null || true
    [[ -z "$SIM_PID" ]] || wait "$SIM_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Launching Gazebo, navigation, camera/LiDAR bridges, YOLO and task manager..."
ros2 launch warehouse_simulation warehouse_full_system.launch.py &
SIM_PID=$!

sleep 4
echo "Launching dashboard..."
ros2 run warehouse_bringup roboverse_dashboard &
DASH_PID=$!

sleep 2
echo
echo "Open http://localhost:8080 in your browser."
if ! python3 -c "import torch, ultralytics" >/dev/null 2>&1; then
    echo "WARNING: YOLO Python dependencies are missing; camera may work without detections."
fi
echo "Keep this terminal open; press Ctrl+C to stop."
wait "$SIM_PID"

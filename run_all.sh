#!/bin/bash
# Run the complete Light-Level Monitoring System
# Run: chmod +x run_all.sh && ./run_all.sh

echo "=========================================="
echo "Light-Level Monitoring System"
echo "=========================================="

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Activate conda environment
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate ros2env

# Source ROS2 workspace
source "$SCRIPT_DIR/ros2_ws/install/setup.bash"

# Start ROS2 serial node in background
echo ""
echo "[1/2] Starting ROS2 Serial Node..."
ros2 run light_monitor serial_node &
ROS2_PID=$!
sleep 2

# Start Flask app
echo ""
echo "[2/2] Starting Flask Web Server..."
echo ""
echo "=========================================="
echo "Open your browser: http://127.0.0.1:1337"
echo "Press Ctrl+C to stop"
echo "=========================================="
echo ""

cd "$SCRIPT_DIR/flask_app"
python app.py

# Cleanup on exit
kill $ROS2_PID 2>/dev/null

#!/bin/bash
# Setup script for Light-Level Monitoring System
# Run: chmod +x setup.sh && ./setup.sh

echo "=========================================="
echo "Light-Level Monitoring System - Setup"
echo "=========================================="

# Check for conda
if ! command -v conda &> /dev/null; then
    echo "ERROR: Conda not found!"
    echo "Install Miniconda from: https://docs.conda.io/en/latest/miniconda.html"
    exit 1
fi

# Create conda environment
echo ""
echo "[1/4] Creating conda environment (ros2env)..."
conda env create -f environment.yml -y 2>/dev/null || conda env update -f environment.yml

# Activate environment
echo ""
echo "[2/4] Activating environment..."
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate ros2env

# Build ROS2 workspace
echo ""
echo "[3/4] Building ROS2 workspace..."
cd ros2_ws
colcon build --symlink-install
cd ..

# Setup MySQL database
echo ""
echo "[4/4] Setting up MySQL database..."
echo "Please enter your MySQL root password when prompted:"
mysql -h 127.0.0.1 -u root -p < db.sql

echo ""
echo "=========================================="
echo "Setup complete!"
echo "=========================================="
echo ""
echo "To run the system:"
echo "  1. Connect your Arduino with the light sensor"
echo "  2. Run: ./run_all.sh"
echo "  3. Open browser: http://127.0.0.1:1337"
echo ""

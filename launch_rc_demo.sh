#!/bin/bash
# RC Car Demo Launch Script
# This script launches the RC car in the narrow corridor world with all sensors

echo "========================================="
echo "RC Car Corridor Demo"
echo "========================================="
echo ""
echo "This will launch:"
echo "  1. Gazebo Ignition with narrow corridor world"
echo "  2. RC car with camera + LiDAR sensors"
echo "  3. Stamper bridge for teleop control"
echo "  4. RViz for visualization"
echo ""
echo "After launch, open a new terminal and run:"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo "Controls: i=forward, j=left, l=right, k=stop, ,=backward"
echo ""
echo "========================================="
echo ""

# Source the workspace
cd ~/tb3_ws/NCHSB_MPPI/NCHSB
source install/setup.bash

# Launch the corridor demo
ros2 launch rc_model_description corridor_bringup.launch.py

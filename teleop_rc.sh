#!/bin/bash
# Teleop Control Script for RC Car
# Run this in Terminal 3 after launching demo and stamper bridge

echo "========================================="
echo "RC Car Teleop Control"
echo "========================================="
echo ""
echo "Controls:"
echo "  i = Move forward"
echo "  j = Turn left"
echo "  l = Turn right"
echo "  k = Stop"
echo "  , = Move backward"
echo ""
echo "  q/z = Increase/decrease max speeds"
echo "  w/x = Increase/decrease linear speed only"
echo "  e/c = Increase/decrease angular speed only"
echo ""
echo "Press Ctrl+C to quit"
echo "========================================="
echo ""

# Source the workspace
cd ~/tb3_ws/NCHSB_MPPI/NCHSB
source install/setup.bash

# Run teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

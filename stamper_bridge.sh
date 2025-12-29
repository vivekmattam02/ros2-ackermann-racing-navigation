#!/bin/bash
# Stamper Bridge Script for RC Car
# Run this in Terminal 2 after launching the RC car demo
# This converts Twist messages to TwistStamped for the Ackerman controller

echo "========================================="
echo "RC Car Stamper Bridge"
echo "========================================="
echo ""
echo "Starting stamper bridge..."
echo "Converts: /cmd_vel (Twist) → /ackermann_steering_controller/reference (TwistStamped)"
echo ""
echo "Leave this running and open another terminal for teleop control"
echo "========================================="
echo ""

# Source the workspace
cd ~/tb3_ws/NCHSB_MPPI/NCHSB
source install/setup.bash

# Run stamper bridge
ros2 run rc_nav_bridge stamper --ros-args -r cmd_vel:=/cmd_vel -r reference:=/ackermann_steering_controller/reference

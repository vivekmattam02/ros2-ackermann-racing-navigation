# Ackermann Narrow Corridor Navigation

High-speed autonomous navigation for Ackermann-steered vehicles in narrow indoor corridors using ROS 2.

## Overview

This project develops a complete navigation framework for an Ackermann RC car operating in corridors as narrow as 1.2 meters. The system combines racing line optimization, camera-LiDAR perception, and MPPI control to achieve efficient high-speed navigation in constrained environments.

## Key Features

- **Custom Ackermann Simulation**: Complete URDF model with Gazebo Ignition integration
- **Racing Line Optimization**: Outside-inside-outside cornering using 92% of corridor width
- **Camera-LiDAR Fusion**: Optical flow with ego-motion compensation for obstacle velocity estimation
- **Full Nav2 Integration**: SLAM Toolbox, EKF localization, MPPI controller

## System Architecture

```
Gazebo Simulation → ros_gz_bridge → Localization (EKF + SLAM) 
                                   ↓
                          Perception (Camera + LiDAR)
                                   ↓
                          Nav2 (Racing Line + MPPI)
                                   ↓
                          Ackermann Steering Controller
```

## Tech Stack

- **ROS 2**: Humble
- **Simulator**: Gazebo Ignition Fortress
- **Navigation**: Nav2, SLAM Toolbox
- **Control**: MPPI, ros2_control
- **Perception**: OpenCV, optical flow
- **Languages**: Python, C++

## Quick Start

### Prerequisites
```bash
# ROS 2 Humble
# Gazebo Ignition Fortress
# Nav2
# SLAM Toolbox
```

### Build
```bash
cd ~/tb3_ws/NCHSB_MPPI/NCHSB
colcon build
source install/setup.bash
```

### Run Simulation
```bash
# Launch full system
./launch_rc_demo.sh

# In another terminal - teleoperation
./teleop_rc.sh
```

## Project Structure

```
NCHSB/
├── src/
│   ├── rc_model_description/    # URDF, worlds, launch files
│   ├── rc_nav_bridge/           # TwistStamper node
│   └── rc_racing_planner/       # Racing line planner
├── config/                      # Nav2, SLAM, EKF configs
├── launch_rc_demo.sh           # Main launch script
└── teleop_rc.sh                # Teleoperation script
```

## Hardware Platform

**Target**: Traxxas RC Car
- Width: 0.28m
- Wheelbase: 0.1869m
- Min turn radius: 0.35m

## Team

- **Nishant Pushparaju** - Path Planning
- **Vivekananda Swamy Mattam** - Perception & Integration
- **Samyu Kamtam** - Control Systems

**Affiliation**: NYU Tandon School of Engineering  
**Funding**: Bell Labs

## Citation

If you use this work, please cite:
```
@inproceedings{pushparaju2025ackermann,
  title={Planning and Control in Narrow Indoor Corridors},
  author={Pushparaju, Nishant and Mattam, Vivekananda Swamy and Kamtam, Samyu},
  year={2025},
  organization={NYU Tandon School of Engineering}
}
```

## License

MIT License - see LICENSE file for details

## Acknowledgments

Special thanks to Dr. Aliasghar Arab for project guidance.

# ROS2 Ackermann Racing Navigation

High-speed autonomous navigation for Ackermann-steered vehicles in narrow indoor corridors using ROS 2.

## The Problem

Indoor navigation in hospitals, warehouses, and office buildings requires robots to move through corridors as narrow as 1.2 meters. Existing solutions are either high-speed racers designed for wide tracks (3-5m) or slow conservative navigators that creep through spaces. Neither handles narrow corridors at useful speeds.

This project bridges that gap.

## Overview

We develop a complete navigation framework for an Ackermann RC car targeting corridor widths of 1.2-1.5 meters. The system combines racing line optimization, camera-LiDAR perception, and MPPI control. All development follows a simulation-first approach using Gazebo Ignition before hardware deployment.

## Key Features

- **Custom Ackermann Simulation**: Complete URDF model solving the four-bar linkage problem, integrated with Gazebo Ignition and ros2_control
- **Racing Line Optimization**: Outside-inside-outside cornering geometry using 92% of corridor width for smoother, faster turns
- **Camera-LiDAR Fusion**: Optical flow with ego-motion compensation for estimating obstacle velocity relative to the robot
- **Full Nav2 Integration**: SLAM Toolbox for mapping, EKF for sensor fusion, MPPI controller with Ackermann motion model

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     Gazebo Ignition Simulation                  │
│  (Ackermann Physics, 90° LiDAR, Camera, IMU)                   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                        ros_gz_bridge                            │
│  (Sensor topics, TF, Clock synchronization)                    │
└─────────────────────────────────────────────────────────────────┘
                              │
              ┌───────────────┴───────────────┐
              ▼                               ▼
┌─────────────────────────┐     ┌─────────────────────────┐
│   Localization Layer    │     │    Perception Layer     │
│  (EKF 50Hz + SLAM 2Hz)  │     │  (Camera-LiDAR Fusion)  │
└─────────────────────────┘     └─────────────────────────┘
              │                               │
              └───────────────┬───────────────┘
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                  Nav2 Navigation Stack                          │
│  (Racing Line Planner, MPPI Controller, Costmaps)              │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│              Ackermann Steering Controller (100Hz)              │
│  (ros2_control, differential steering angles)                  │
└─────────────────────────────────────────────────────────────────┘
```

## Tech Stack

| Component | Technology |
|-----------|------------|
| Framework | ROS 2 Humble |
| Simulator | Gazebo Ignition Fortress |
| Navigation | Nav2, SLAM Toolbox |
| Localization | robot_localization (EKF) |
| Control | MPPI, ros2_control |
| Perception | OpenCV (Farneback optical flow) |
| Languages | Python, C++ |

## Quick Start

### Prerequisites
- ROS 2 Humble
- Gazebo Ignition Fortress
- Nav2
- SLAM Toolbox
- robot_localization

### Build
```bash
cd ~/tb3_ws/NCHSB_MPPI/NCHSB
colcon build
source install/setup.bash
```

### Run Simulation
```bash
# Terminal 1: Launch full system
./launch_rc_demo.sh

# Terminal 2: Teleoperation
./teleop_rc.sh
```

## Project Structure

```
NCHSB/
├── src/
│   ├── rc_model_description/     # URDF, meshes, worlds, launch files
│   │   ├── urdf/                 # Ackermann URDF/XACRO files
│   │   ├── worlds/               # Narrow corridor SDF world
│   │   ├── config/               # Controller, EKF, SLAM configs
│   │   └── launch/               # Bringup launch files
│   ├── rc_nav_bridge/            # TwistStamper node for Ackermann
│   └── rc_racing_planner/        # Racing line global planner
├── launch_rc_demo.sh             # Main launch script
├── teleop_rc.sh                  # Teleoperation script
└── stamper_bridge.sh             # TwistStamped converter
```

## Hardware Platform

**Target Platform**: Traxxas RC Car with Ackermann Steering

| Parameter | Value |
|-----------|-------|
| Width | 0.28m |
| Wheelbase | 0.1869m |
| Front Track | 0.137m |
| Rear Track | 0.145m |
| Wheel Radius | 0.055m |
| Max Steering Angle | ±28° |
| Min Turn Radius | 0.35m (theoretical) |

## System Specifications

| Component | Specification |
|-----------|---------------|
| EKF Update Rate | 50 Hz |
| SLAM Resolution | 0.05 m/cell |
| MPPI Samples | 2000 trajectories |
| MPPI Horizon | 2.8s (56 timesteps) |
| Ackermann Controller | 100 Hz |
| Perception Processing | 20-25 fps |
| Total ROS2 Nodes | 60+ |

## Team

| Name | Role |
|------|------|
| Nishant Pushparaju | Control Systems |
| Vivekananda Swamy Mattam | Perception and Integration |
| Samyu Kamtam | Path Planning |

**Affiliation**: NYU Tandon School of Engineering, Department of Mechanical and Aerospace Engineering  
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

Special thanks to Dr. Aliasghar Arab for project guidance and technical mentorship throughout the development period.

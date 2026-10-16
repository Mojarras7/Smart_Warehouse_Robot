# Smart Warehouse AMR: Full-Stack Autonomous Logistics System

Autonomous mobile robotics platform combining a Gazebo digital twin simulation in ROS 2 Jazzy with physical Ackermann vehicle deployment and distributed YOLOv8 perception in ROS 2 Foxy.

## Monorepo Architecture & Provenance

This repository consolidates the full lifecycle of the Smart Warehouse AMR project into a single engineering workspace. It unifies two core upstream repositories developed collaboratively:

- **Physical Platform Stack**: [antoniofp/smart_warehouse_robot](https://github.com/antoniofp/smart_warehouse_robot) provides the hardware drivers, CycloneDDS network tuning, Nav2 Pure Pursuit configuration, and mission state machine for the NVIDIA Jetson Nano onboard the Rosmaster R2 chassis.
- **Simulation & Digital Twin**: [Mojarras7/yahboomR2_sim](https://github.com/Mojarras7/yahboomR2_sim) provides the URDF model descriptions, mesh assets, Gazebo Sim sensor plugins, and warehouse world environments in ROS 2 Jazzy.

Developers and evaluators can clone this monorepo directly to inspect the complete full-stack implementation, or utilize the provided [warehouse.repos](warehouse.repos) manifest with the ROS 2 `vcstool` utility to pull individual upstream repositories.

## System Architecture

The project implements a distributed three-tier architecture that addresses the computational constraints of edge robotics:

```
┌─────────────────────────────────────────────────────────────┐
│                    TIER 1: HOST PC (GPU)                    │
│  - Offboard YOLOv8 Sign Detection (CUDA, 320x320 imgsz)     │
│  - HTTP Video Stream Ingestion (:8089)                      │
│  - Low-Latency UDP Telemetry Transmission (:5005)           │
└──────────────▲──────────────────────────────┬───────────────┘
               │ HTTP MJPEG Stream            │ UDP Telemetry
┌──────────────┴──────────────────────────────▼───────────────┐
│              TIER 2: ONBOARD ROBOT (JETSON NANO)            │
│  Ubuntu 20.04 + ROS 2 Foxy (Docker: beautiful_snyder)       │
│                                                             │
│  - Mission State Machine (mission_vision.py)                │
│  - SLAM Toolbox (Localization Mode)                         │
│  - Nav2 Navigation Stack (Regulated Pure Pursuit)           │
│  - Yahboom Ackermann Base Driver & RPLidar A1               │
│  - Embedded HTTP Video Streamer (:8089)                     │
│  - Hardware Steering Trim (offset=0.0, scale=0.78)          │
└─────────────────────────────────────────────────────────────┘
                               ▲
                 Sim-to-Real   │ (URDF / Sensor Specifications)
                               ▼
┌─────────────────────────────────────────────────────────────┐
│                 TIER 3: DIGITAL TWIN (SIMULATION)           │
│  Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Sim                    │
│                                                             │
│  - Parameterized URDF / Xacro Model (master_R2.urdf.xacro)  │
│  - Multi-Sensor Bridge (ros_gz_bridge)                      │
│  - Warehouse Environment Model (warehouse.sdf)              │
│  - Online SLAM Mapping & RViz Visualization                 │
└─────────────────────────────────────────────────────────────┘
```

## Workspace Structure

```text
SmartWarehouse/
├── warehouse.repos           # VCS manifest linking upstream repositories
├── smart_warehouse_robot/    # Physical platform deployment workspace (ROS 2 Foxy)
│   ├── scripts/              # Mission state machine and offboard vision node
│   ├── src/r2_nav/           # Nav2 configurations, maps, and behavior trees
│   ├── src/slam_toolbox/     # 2D SLAM and localization package
│   ├── src/yolo_pkg/         # ROS 2 perception wrapper nodes
│   ├── utils/                # Calibration loggers, LiDAR fix, and diagnostic tools
│   ├── start_*.sh            # Operational launch and lifecycle management scripts
│   └── docs/                 # Jetson deployment guide and architecture notes
└── yahboomR2_sim/            # Simulation and digital twin workspace (ROS 2 Jazzy)
    ├── docs/                 # Documentation media, screenshots, and demonstration videos
    ├── requirements.txt      # System package requirements for ROS 2 Jazzy
    └── yahboomcar_description/
        ├── config/           # SLAM Toolbox online async mapping parameters
        ├── launch/           # gazebo_R2.launch.py simulation launcher
        ├── meshes/           # STL geometric chassis and wheel models
        ├── urdf/             # Parametric Xacro descriptions and Gazebo plugins
        └── worlds/           # SDF simulation environments (empty_world, warehouse)
```

## Workspace Setup & Synchronization

### Option A: Standard Clone (Monorepo with Submodules)

Clone the repository and recursively initialize all submodules:

```bash
git clone --recurse-submodules https://github.com/Mojarras7/Smart_Warehouse_Robot.git
cd SmartWarehouse
```

### Option B: ROS 2 VCS Tool Setup

If maintaining individual repository remotes inside a ROS 2 workspace:

```bash
mkdir -p ~/warehouse_ws/src
cd ~/warehouse_ws
vcs import src < /path/to/warehouse.repos
```

## Quick Start Guide

### 1. Running the Digital Twin Simulation

Prerequisites: Ubuntu 24.04 with ROS 2 Jazzy.

```bash
# Install dependencies
cd yahboomR2_sim
xargs -a requirements.txt sudo apt install -y

# Build the description package
colcon build --packages-select yahboomcar_description
source install/setup.bash

# Launch warehouse world with SLAM and RViz
ros2 launch yahboomcar_description gazebo_R2.launch.py world:=warehouse.sdf
```

### 2. Running the Physical Robot Stack

Prerequisites: NVIDIA Jetson Nano connected to local network.

```bash
# Access the onboard Docker runtime
docker start beautiful_snyder
docker exec -it beautiful_snyder bash

# Source workspace environments
source /opt/ros/foxy/setup.bash
source /root/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
source /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
source /root/smart_warehouse_robot/install/setup.bash

# Configure CycloneDDS network middleware
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=32

# Launch autonomous navigation and mission server
./start_mission.sh
```

### 3. Running Offboard Vision Inference

On a host workstation equipped with an NVIDIA GPU:

```bash
cd smart_warehouse_robot
pip install ultralytics torch opencv-python numpy

# Direct inference to the robot IP address
export JETSON_IP="192.168.1.100"
python3 scripts/pc_vision_node.py
```

## Warehouse Behavioral State Machine

The mission controller processes classification telemetry received over UDP and dynamically modifies motion commands:

| Sign Trigger | Operational Mode | Execution Details |
| :--- | :--- | :--- |
| `pedestrian_zone` | Outbound Transit | Throttles linear velocity by 50% to navigate safely near warehouse personnel. |
| `restricted_zone` | Outbound Transit | Aborts current target waypoint and commands Nav2 to plan an alternate route. |
| `loading_zone` | Outbound Terminal | Halts vehicle at dock and awaits operator confirmation via `./send_continue.sh`. |
| Docking Maneuver | Route Transition | Executes automated reverse maneuver to reorient chassis for the return run. |
| `stop_for_safety` | Inbound Transit | Enforces immediate 5.0-second standstill before resuming progress. |
| `robot_only_zone` | Inbound Transit | Restores nominal maximum velocity to 100%. |
| `parking_zone` | Mission End | Engages motor brakes and coordinates clean process shutdown. |

## Hardware Calibration

Ackermann steering requires angular servo calibration to ensure odometric accuracy:

- `steering_offset`: Neutral steering trim in degrees.
- `linear_scale_y`: Odometry angular scale correction factor (`0.78`).

Parameters are applied during bringup in [start_navigation.sh](smart_warehouse_robot/start_navigation.sh):

```bash
ros2 launch yahboomcar_nav laser_bringup_launch.py steering_offset:=0.0 linear_scale_y:=0.78
```

## Authors & Contributors

- **Alejandro Mojarras** ([@Mojarras7](https://github.com/Mojarras7)) — Simulation Lead, Gazebo Digital Twin, SLAM Toolbox implementation, and YOLOv8 training.
- **Manuel Arámbula** ([@Marambulag](https://github.com/Marambulag)) — Mission state machine logic, offboard vision pipeline, and warehouse validation.
- **J. Antonio Fernández** ([@antoniofp](https://github.com/antoniofp)) — Physical platform host, hardware bringup, and Kalman filter debugging.


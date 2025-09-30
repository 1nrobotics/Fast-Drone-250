# PX4Ctrl - Detailed Technical Documentation

## Overview

PX4Ctrl is a sophisticated flight controller interface designed for PX4-based multirotor drones. It provides a high-level control layer that bridges trajectory planning systems with low-level PX4 flight control, enabling autonomous flight operations with comprehensive safety mechanisms.

## Table of Contents

1. [System Architecture](#system-architecture)
2. [Main Components](#main-components)
3. [Communication Flow](#communication-flow)
4. [Finite State Machine](#finite-state-machine)
5. [Control System](#control-system)
6. [Safety Mechanisms](#safety-mechanisms)
7. [Configuration](#configuration)
8. [Message Structures](#message-structures)
9. [Key Features](#key-features)

## System Architecture

The PX4Ctrl system follows a modular architecture with clear separation of concerns:

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Ego-Planner   │───▶│   Traj_Server    │───▶│    PX4Ctrl      │
│  (Path Planning)│    │ (Traj Execution) │    │ (Flight Control)│
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                        │                        │
         ▼                        ▼                        ▼
  B-Spline Trajectory    Position Commands         Control Output
```

### Core Components

1. **px4ctrl_node.cpp** - Main ROS node and entry point
2. **PX4CtrlFSM.cpp/.h** - Finite State Machine managing flight modes
3. **controller.cpp/.h** - Linear control algorithms
4. **input.cpp/.h** - Input data handling and processing
5. **PX4CtrlParam.cpp/.h** - Parameter management system

## Main Components

### Main Node (`px4ctrl_node.cpp`)

The main node serves as the central hub that:

#### Initialization
- Creates a `Parameter_t` object and loads configuration from ROS parameters
- Instantiates a `LinearControl` controller and `PX4CtrlFSM` state machine
- Sets up signal handler for graceful shutdown

#### ROS Interface Setup

**Subscribers:**
- `/mavros/state` - Flight controller state (armed/disarmed, flight mode)
- `/mavros/extended_state` - Extended state info (landing state)
- `~odom` - Odometry data (remapped to `/vins_fusion/imu_propagate`)
- `~cmd` - Position commands (remapped to `/position_cmd`)
- `/mavros/imu/data` - IMU data for attitude estimation
- `/mavros/rc/in` - Remote controller inputs (optional)
- `/mavros/battery` - Battery status
- `takeoff_land` - Takeoff/land command topic

**Publishers:**
- `/mavros/setpoint_raw/attitude` - Attitude control commands to PX4
- `/traj_start_trigger` - Triggers trajectory execution
- `/debugPx4ctrl` - Debug information

**Service Clients:**
- `/mavros/set_mode` - Change flight modes
- `/mavros/cmd/arming` - Arm/disarm motors
- `/mavros/cmd/command` - Send commands to FCU

#### Main Loop
- Runs at configurable frequency (`ctrl_freq_max`, typically 100Hz)
- Calls `fsm.process()` continuously to execute the state machine logic

### Finite State Machine (`PX4CtrlFSM`)

The FSM manages five distinct flight states:

#### State Definitions
```cpp
enum class FlightState : uint8_t {
    ManualControl = 1,  // RC control only
    AutoHover,          // Autonomous hovering
    CommandControl,     // Following position commands  
    AutoTakeoff,        // Automated takeoff sequence
    AutoLand            // Automated landing sequence
};
```

#### State Transition Diagram
```
    system start
          │
          ▼
    ┌──────────────┐
    │ManualControl │◄─────────────┐
    └──────┬───────┘              │
           │                      │
           ▼                      │
    ┌──────────────┐              │
    │  AutoHover   │              │
    └──┬────────┬──┘              │
       │        │                 │
       ▼        ▼                 │
┌─────────┐ ┌─────────┐          │
│AutoTakeoff│ │AutoLand │──────────┘
└─────────┘ └─────────┘
       │
       ▼
┌──────────────┐
│CommandControl│
└──────────────┘
```

#### Safety Mechanisms
- **Velocity checks** - Rejects mode changes if drone moving too fast
- **Command validation** - Ensures no conflicting commands during transitions  
- **Timeout protection** - Falls back to safe modes if data streams fail
- **RC override** - Manual control always available as fallback

## Communication Flow

### Planner to PX4Ctrl Communication

```
EGO-Planner → Traj_Server → PX4Ctrl
     ↓             ↓           ↓
  B-Spline    PositionCommand  Control
 Trajectory      Message      Output
```

#### Step-by-Step Process:

1. **EGO-Planner** generates B-spline trajectory and publishes to `planning/bspline`
2. **Trajectory Server** receives B-spline, converts to position commands at 100Hz
3. **PX4Ctrl** receives position commands via `/position_cmd` topic
4. **FSM** processes commands and generates control outputs
5. **Controller** sends attitude commands to PX4 via MAVROS

#### Detailed Position Command Flow:

**A. B-Spline to Position Command Conversion:**
The trajectory server (`src/planner/plan_manage/src/traj_server.cpp`) performs real-time conversion:
- Subscribes to `/planning/bspline` topic from EGO-Planner
- Evaluates B-spline using De Boor algorithm at 100Hz (10ms timer)
- Calculates derivatives for velocity and acceleration
- Computes yaw angle from trajectory direction with look-ahead
- Publishes `quadrotor_msgs::PositionCommand` to `/position_cmd`

**B. PositionCommand Message Structure:**
```cpp
// quadrotor_msgs/PositionCommand.msg
Header header                    // Timestamp and coordinate frame
geometry_msgs/Point position     // Target position (x,y,z) [m]
geometry_msgs/Vector3 velocity   // Target velocity (vx,vy,vz) [m/s]
geometry_msgs/Vector3 acceleration // Target acceleration (ax,ay,az) [m/s²]
geometry_msgs/Vector3 jerk       // Target jerk (jx,jy,jz) [m/s³]
float64 yaw                      // Target yaw angle [rad]
float64 yaw_dot                  // Target yaw rate [rad/s]
float64[3] kx, kv               // Optional position/velocity gains
uint32 trajectory_id            // Trajectory identifier
uint8 trajectory_flag           // Status (READY, COMPLETED, etc.)
```

**C. PX4Ctrl Command Reception:**
```cpp
// From px4ctrl_node.cpp - ROS subscriber setup
ros::Subscriber cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>(
    "cmd", 100, boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1),
    ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

// From input.cpp - Command processing
void Command_Data_t::feed(quadrotor_msgs::PositionCommandConstPtr pMsg) {
    msg = *pMsg;
    rcv_stamp = ros::Time::now();  // Record reception time
    
    // Extract position/velocity/acceleration/jerk vectors
    p << msg.position.x, msg.position.y, msg.position.z;
    v << msg.velocity.x, msg.velocity.y, msg.velocity.z;
    a << msg.acceleration.x, msg.acceleration.y, msg.acceleration.z;
    j << msg.jerk.x, msg.jerk.y, msg.jerk.z;
    
    yaw = uav_utils::normalize_angle(msg.yaw);  // Normalize to [-π,π]
}
```

**D. Communication Parameters:**
- **Frequency**: 100Hz position command generation and publishing
- **Latency**: TCP transport with `tcpNoDelay()` for minimal delay  
- **Buffering**: 100-message queue size for real-time performance
- **Topics**: `/planning/bspline` → `/position_cmd` → PX4Ctrl processing
- **Timeout**: Commands must arrive within specified timeout period

### Message Flow Details

#### 1. B-Spline Publication (EGO-Planner)
```cpp
// Location: src/planner/plan_manage/src/ego_replan_fsm.cpp
traj_utils::Bspline bspline;
bspline.order = 3;
bspline.start_time = info->start_time_;
bspline.traj_id = info->traj_id_;

// Control points and knot vector setup
bspline_pub_.publish(bspline);
```

#### 2. Command Generation (Trajectory Server)
```cpp
// Location: src/planner/plan_manage/src/traj_server.cpp
// 100Hz command generation from B-spline
pos = traj_[0].evaluateDeBoorT(t_cur);    // Position
vel = traj_[1].evaluateDeBoorT(t_cur);    // Velocity  
acc = traj_[2].evaluateDeBoorT(t_cur);    // Acceleration

cmd.position.x = pos(0); cmd.position.y = pos(1); cmd.position.z = pos(2);
cmd.velocity.x = vel(0); cmd.velocity.y = vel(1); cmd.velocity.z = vel(2);
cmd.acceleration.x = acc(0); cmd.acceleration.y = acc(1); cmd.acceleration.z = acc(2);

pos_cmd_pub.publish(cmd);
```

#### 3. Command Processing (PX4Ctrl)
```cpp
// Location: src/realflight_modules/px4ctrl/src/input.cpp
void Command_Data_t::feed(quadrotor_msgs::PositionCommandConstPtr pMsg) {
    // Extract position, velocity, acceleration, jerk
    p(0) = msg.position.x; p(1) = msg.position.y; p(2) = msg.position.z;
    v(0) = msg.velocity.x; v(1) = msg.velocity.y; v(2) = msg.velocity.z;
    // ... process all command data
}
```

## Control System

### Linear Controller (`LinearControl`)

Implements a **cascade PID controller** with the following structure:

#### Control Law
```cpp
// Desired acceleration calculation
des_acc = des.a + Kv.asDiagonal() * (des.v - odom.v) + Kp.asDiagonal() * (des.p - odom.p);
des_acc += Vector3d(0,0,grav);  // Gravity compensation
```

#### Attitude Generation
- Computes desired **roll/pitch** from lateral acceleration demands
- Uses current yaw from odometry for reference frame alignment
- Transforms through IMU orientation to account for sensor alignment

#### Thrust Mapping
- Converts vertical acceleration to normalized thrust signal
- Includes adaptive thrust-to-acceleration mapping (`thr2acc_`)
- Supports both accurate and approximate thrust models

#### Control Gains
- **Kp** - Position gains (X, Y, Z axes)
- **Kv** - Velocity gains (X, Y, Z axes) 
- Configurable per-axis for different dynamics

## Safety Mechanisms

### Multiple Safety Layers

1. **State Validation**
   - Velocity limits checking
   - Acceleration bounds verification
   - Tilt angle restrictions

2. **Timeout Protection**
   - Sensor data freshness monitoring
   - Command stream continuity checking
   - Automatic fallback to safe states

3. **Emergency Procedures**
   - RC override capabilities
   - Emergency landing sequences
   - Motor disarm protection

4. **Input Validation**
   - Range checking for all commands
   - Sanity checks on sensor data
   - Cross-validation between sensors

## Configuration

### Key Parameters (`ctrl_param_fpv.yaml`)

#### Physical Properties
- `mass: 1.2` - Vehicle mass (kg)
- `gra: 9.81` - Gravitational acceleration

#### Control Gains
- `Kp0/1/2: 1.5` - Position control gains  
- `Kv0/1/2: 1.5` - Velocity control gains

#### Thrust Model
- `K1, K2, K3` - Thrust mapping coefficients
- `hover_percentage: 0.30` - Nominal hover throttle

#### Safety Limits
- `max_angle: 30` - Maximum tilt angle (degrees)
- `low_voltage: 13.2` - Battery warning threshold
- `max_manual_vel: 1.0` - Manual mode speed limit

#### Timeouts
- Message timeout values for all sensor inputs
- Used for failsafe detection

## Message Structures

### PositionCommand Message
```cpp
Header header                    // Timestamp and frame
geometry_msgs/Point position    // Desired position (x,y,z)
geometry_msgs/Vector3 velocity  // Desired velocity (x,y,z)  
geometry_msgs/Vector3 acceleration // Desired acceleration (x,y,z)
geometry_msgs/Vector3 jerk      // Desired jerk (x,y,z)
float64 yaw                     // Desired yaw angle
float64 yaw_dot                 // Desired yaw rate
float64[3] kx                   // Position gains (optional)
float64[3] kv                   // Velocity gains (optional)
uint32 trajectory_id            // Trajectory identifier
uint8 trajectory_flag           // Status flags
```

### Controller Output
```cpp
struct Controller_Output_t {
    Eigen::Quaterniond q;       // Orientation of body frame
    Eigen::Vector3d bodyrates;  // Body rates in body frame [rad/s]
    double thrust;              // Collective mass normalized thrust
};
```

## Key Features & Design Principles

### Advantages

1. **Decoupling** - Planner and controller are separated by trajectory server
2. **Real-time** - 100Hz position command generation from trajectory
3. **Smooth execution** - B-spline provides C² continuity 
4. **Feedforward control** - Full state reference (pos, vel, acc, jerk)
5. **Yaw management** - Automatic yaw calculation based on velocity direction

### Design Principles

#### Modularity
- Clean separation between state management, control, and I/O
- Configurable parameters without code changes
- Extensible controller interface

#### Safety-First Design  
- Multiple redundant failsafes and timeout mechanisms
- Graceful degradation to manual control
- Extensive input validation and safety checks

#### Real-Time Performance
- Efficient state machine with minimal computational overhead  
- Optimized message processing with TCP no-delay hints
- High-frequency control loop (100Hz) for responsive control

#### Integration with PX4
- Native MAVROS integration for seamless PX4 communication
- Supports PX4's offboard mode and attitude control interface
- Compatible with PX4's safety features and failsafes

## Build System & Dependencies

### Dependencies
- **ROS packages**: `roscpp`, `geometry_msgs`, `sensor_msgs`, `mavros`
- **Custom messages**: `quadrotor_msgs` (position commands, debug data)
- **Utilities**: `uav_utils` (coordinate transformations, utilities)
- **Linear algebra**: Eigen3 for mathematical operations

### Compilation
- C++11 standard with optimized release build (`-O3`)
- Single executable: `px4ctrl_node`
- Python thrust calibration scripts included

## Usage

### Basic Operation Sequence
1. Launch MAVROS connection to PX4: `roslaunch mavros px4.launch`
2. Start VINS-Fusion for state estimation: `sh shfiles/rspx4.sh`
3. Launch PX4Ctrl: `roslaunch px4ctrl run_ctrl.launch`
4. Start trajectory planner: `roslaunch ego_planner single_run_in_exp.launch`

### Emergency Procedures
- **Case 1**: Planning issues → Switch to hover mode (channel 6)
- **Case 2**: VINS failure → Manual control (channel 5)  
- **Case 3**: Collision → Assess and land safely
- **Case 4**: Emergency → Kill switch (channel 7)

---

**Note**: This documentation reflects the improved codebase with modern C++ conventions, proper namespacing, and comprehensive safety mechanisms as implemented in the recent refactoring.
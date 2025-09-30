# PX4Ctrl System Overview

## What is PX4Ctrl?

PX4Ctrl is a high-level flight controller interface for PX4-based multirotor drones that enables autonomous flight operations. It acts as a bridge between trajectory planning systems (like Ego-Planner) and the low-level PX4 flight controller.

## System Architecture

```
Trajectory Planner → PX4Ctrl → PX4 Flight Controller → Motors
       ↓               ↓              ↓                ↓
   Path Planning   State Machine   Attitude Control   Actuation
```

## Key Components

### 1. Finite State Machine (FSM)
Manages five flight states:
- **Manual Control**: Direct RC control
- **Auto Hover**: Autonomous hovering with position hold
- **Command Control**: Following trajectory commands
- **Auto Takeoff**: Automated takeoff sequence
- **Auto Land**: Automated landing sequence

### 2. Linear Controller
- Cascade PID controller for position and attitude control
- Feedforward control with position, velocity, and acceleration commands
- Adaptive thrust mapping with battery voltage compensation

### 3. Safety System
- Multiple timeout protections for sensor data
- RC override capabilities in all autonomous modes
- Velocity and acceleration limit enforcement
- Emergency landing procedures

### 4. Communication Interface
- Receives trajectory commands via ROS topics
- Publishes attitude commands to PX4 via MAVROS
- Handles sensor data from IMU, odometry, and RC

## Communication Flow

### From Planner to Control
1. **Ego-Planner** generates B-spline trajectory → publishes to `/planning/bspline`
2. **Trajectory Server** converts B-spline to 100Hz position commands → publishes to `/position_cmd`
3. **PX4Ctrl** subscribes to `/position_cmd` and processes through state machine
4. **Controller** generates attitude/thrust commands → sends to PX4 via MAVROS
5. **PX4** executes low-level attitude control and motor commands

### Key ROS Topics
- **B-Spline Input**: `/planning/bspline` (from EGO-Planner)
- **Position Commands**: `/position_cmd` (from Trajectory Server to PX4Ctrl)
- **Attitude Output**: `/mavros/setpoint_raw/attitude` (to PX4)
- **Telemetry**: `/odom`, `/mavros/imu/data`, `/mavros/rc/in`

### Message Types
- **Input**: `quadrotor_msgs::PositionCommand` (pos, vel, acc, jerk, yaw)
- **Output**: `mavros_msgs::AttitudeTarget` (quaternion, thrust)
- **Trajectory**: `traj_utils::Bspline` (control points, knots, timing)
- **Telemetry**: Odometry, IMU, battery, RC data

## Configuration

Key parameters in `ctrl_param_fpv.yaml`:
- **Physical**: `mass`, vehicle weight and dynamics
- **Control**: `Kp`, `Kv` gains for position/velocity control  
- **Safety**: `max_angle`, `max_vel` limits
- **Thrust**: `hover_percentage` for takeoff/hover

## Safety Features

### Multi-Layer Protection
1. **Input Validation**: Range and sanity checking
2. **Timeout Monitoring**: Sensor data freshness
3. **State Constraints**: Velocity and acceleration limits
4. **Emergency Procedures**: RC override and kill switches
5. **Graceful Degradation**: Automatic fallback to safe states

### Emergency Response
- **RC Channel 5**: Switch to manual control
- **RC Channel 6**: Switch to hover mode
- **RC Channel 7**: Emergency motor stop
- Automatic landing on critical failures

## Integration

### With PX4 Autopilot
- Uses MAVROS for communication
- Operates in PX4's OFFBOARD mode
- Compatible with standard PX4 safety features
- Supports standard RC failsafes

### With Planning Systems
- Receives B-spline trajectories from planners
- Provides smooth trajectory execution
- Supports real-time replanning
- Maintains C² continuity for smooth flight

## Usage

### Basic Flight Sequence
1. Start MAVROS connection to PX4
2. Launch state estimation (VINS-Fusion)
3. Start PX4Ctrl node
4. Enable offboard mode via RC
5. Launch trajectory planner
6. Execute autonomous flight missions

### Typical Command Flow
```bash
# Terminal 1: Start core systems
sh shfiles/rspx4.sh

# Terminal 2: Start flight controller
roslaunch px4ctrl run_ctrl.launch

# Terminal 3: Start trajectory planner  
roslaunch ego_planner single_run_in_exp.launch

# Terminal 4: Enable autonomous flight
sh shfiles/takeoff.sh
```

## Performance Characteristics

- **Control Frequency**: 100Hz for responsive control
- **State Estimation**: Uses VINS-Fusion at ~30Hz
- **Command Latency**: <10ms from command to actuator
- **Safety Response**: <100ms for emergency procedures
- **Precision**: ±0.1m position accuracy in good conditions

## Recent Improvements

The codebase has been recently refactored with:
- Modern C++ conventions and naming
- Comprehensive documentation
- Enhanced safety mechanisms  
- Proper namespace organization
- Improved code maintainability

---

For detailed technical information, see `PX4CTRL_TECHNICAL_DOCUMENTATION.md`.
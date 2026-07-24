# Ultrasound Scanning Impedance Controller

A Cartesian impedance controller for robotic ultrasound scanning on the Franka FR3. It enables an operator to hand-guide an ultrasound probe across a patient while the robot automatically maintains a constant contact force against the tissue surface.

## Overview

The controller operates in the **flange (probe) frame** and splits the task into two control phases that run simultaneously:

### Phase 1 – Translational Control (Force + Hand-Guiding)

| Axis | Behaviour | Stiffness | Damping |
|------|-----------|-----------|---------|
| **X** (tangent) | Free hand-guiding | 0 N/m | 10 N·s/m |
| **Y** (tangent) | Free hand-guiding | 0 N/m | 10 N·s/m |
| **Z** (normal) | Constant contact force via admittance loop | 400 N/m | 70 N·s/m |

- The tangent axes (X/Y) have zero stiffness so the operator can freely slide the probe across the skin.
- The normal axis (Z) uses an **outer-loop admittance controller** that adjusts the desired position to regulate the contact force to a configurable setpoint (default 5 N).

### Phase 2 – Rotational Control (Selective Compliance)

| Axis | Behaviour | Stiffness | Damping |
|------|-----------|-----------|---------|
| **Roll**  (X) | Free – adapts to body curvature | 0 Nm/rad | 1 Nm·s/rad |
| **Pitch** (Y) | Locked to initial orientation | 30 Nm/rad | 6 Nm·s/rad |
| **Yaw**   (Z) | Locked to initial orientation | 30 Nm/rad | 6 Nm·s/rad |

Roll is left compliant so the probe can conform to the patient's body surface, while pitch and yaw are held to prevent the probe from tilting or spinning during scanning.

### Additional Features

- **Nullspace stabilisation** keeps the joint configuration near the activation pose without affecting Cartesian behaviour.
- **Torque rate limiting** and per-axis wrench saturation for safe operation.
- **Runtime status publisher** outputs a 24-element `Float64MultiArray` for live monitoring of forces, errors, and commands.

## Parameters

All parameters are loaded from YAML configuration. Default values are shown below.

### Translational Gains (Flange Frame)

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `translational_stiffness.x` | 0.0 | N/m | Tangent stiffness (X) |
| `translational_stiffness.y` | 0.0 | N/m | Tangent stiffness (Y) |
| `translational_stiffness.z` | 400.0 | N/m | Normal stiffness (Z) |
| `translational_damping.x` | 10.0 | N·s/m | Tangent damping (X) |
| `translational_damping.y` | 10.0 | N·s/m | Tangent damping (Y) |
| `translational_damping.z` | 70.0 | N·s/m | Normal damping (Z) |

### Rotational Gains (Flange Frame)

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `rotational_stiffness.x` | 0.0 | Nm/rad | Roll – free |
| `rotational_stiffness.y` | 30.0 | Nm/rad | Pitch – locked |
| `rotational_stiffness.z` | 30.0 | Nm/rad | Yaw – locked |
| `rotational_damping.x` | 1.0 | Nm·s/rad | Roll damping |
| `rotational_damping.y` | 6.0 | Nm·s/rad | Pitch damping |
| `rotational_damping.z` | 6.0 | Nm·s/rad | Yaw damping |

### Force Control / Admittance

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `force_normal_desired` | 5.0 | N | Target normal contact force (pressing-positive) |
| `force_filter_cutoff_hz` | 20.0 | Hz | Low-pass filter cutoff on measured force |
| `admittance_gain` | 0.0002 | m/s per N | Admittance velocity per unit force error |
| `normal_v_max` | 0.02 | m/s | Maximum admittance velocity along normal |
| `normal_max_disp` | 0.02 | m | Maximum admittance displacement from initial |

### Nullspace Stabilisation

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `nullspace_stiffness` | 15.0 | Nm/rad | Joint-space stiffness for nullspace control |
| `nullspace_damping` | 3.0 | Nm·s/rad | Joint-space damping for nullspace control |

### Safety Limits

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `max_force_xy` | 30.0 | N | Maximum tangential Cartesian force |
| `max_force_z` | 80.0 | N | Maximum normal Cartesian force |
| `max_torque_xyz` | 20.0 | Nm | Maximum Cartesian torque |
| `max_joint_torque` | 80.0 | Nm | Maximum per-joint torque |
| `max_delta_tau` | 1.0 | Nm | Per-cycle torque rate limit |

### Monitoring

| Parameter | Default | Description |
|-----------|---------|-------------|
| `status_topic` | `ultrasound_scanning_impedance/status` | Topic name for status messages |
| `status_publish_rate_hz` | 20.0 | Publication rate (Hz) |

### Other

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_type` | `fr3` | Robot type identifier |
| `arm_prefix` | `""` | Joint namespace prefix |

## ROS 2 Interfaces

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `ultrasound_scanning_impedance/status` | `std_msgs/msg/Float64MultiArray` | 24-element array with force readings, errors, commands, saturation flags |

### Hardware Interfaces

**State interfaces (read):**
- `{arm_prefix}{robot_type}_joint{1..7}/position`
- `{arm_prefix}{robot_type}_joint{1..7}/velocity`
- Franka robot state (full `franka::RobotState` struct)
- Franka robot model (kinematics and dynamics)

**Command interfaces (write):**
- `{arm_prefix}{robot_type}_joint{1..7}/effort`

## Dependencies

- `controller_interface` – ROS 2 control framework
- `franka_semantic_components` – `FrankaRobotModel` for kinematics/dynamics
- `Eigen3` – linear algebra
- `rclcpp` / `rclcpp_lifecycle` – ROS 2 core
- `pluginlib` – dynamic plugin loading
- `std_msgs` – status publishing
- `franka_msgs` – Franka message types

## Usage

### Clone the latests dependencies
```bash
vcs import src < src/dependency.repos --recursive --skip-existing
```

### Build the workspace
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Launch

```bash
ros2 launch franka_bringup ultrasound_scanning_impedance_controller.launch.py
```

This wraps the shared `example.launch.py` launcher, which brings up the full FR3 hardware stack and spawns the controller via `controller_manager`.

### Configuration

Edit the parameter file at:

```
franka_bringup/config/ultrasound_scanning_impedance_controller.yaml
```

### Controller Plugin

Registered as:

```
franka_example_controllers/UltrasoundScanningImpedanceController
```

Base class: `controller_interface::ControllerInterface`

## File Structure

```
franka_example_controllers/
├── include/franka_example_controllers/fr3/
│   └── ultrasound_scanning_impedance_controller.hpp   # Header
├── src/fr3/
│   └── ultrasound_scanning_impedance_controller.cpp   # Implementation
└── franka_example_controllers.xml                     # Plugin registration

franka_bringup/
├── config/
│   └── ultrasound_scanning_impedance_controller.yaml  # Parameters
└── launch/
    └── ultrasound_scanning_impedance_controller.launch.py  # Launch file
```

## License

Apache License 2.0 – see [LICENSE](../LICENSE) for details.

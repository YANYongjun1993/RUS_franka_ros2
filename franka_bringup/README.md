# franka_bringup

Launch and runtime configuration for the Franka FR3 ultrasound scanning controller.

Configure the robot in `config/franka.config.yaml`, then launch:

```bash
ros2 launch franka_bringup ultrasound_scanning_impedance_controller.launch.py
```

Controller registration and parameters are defined in `config/controllers.yaml`.
The default desired normal force is 5 N.

Runtime status is published as `std_msgs/msg/Float64MultiArray` on:

```bash
/ultrasound_scanning_impedance/status
```

Inspect it with:

```bash
ros2 topic echo /ultrasound_scanning_impedance/status
```

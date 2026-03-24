# franka_bringup 

Launch file package for booting up franka robots.

## Testing

**Note:** To activate the launch file smoke tests, you need to enable the `BUILD_TESTING` CMake flag when building this package:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
```

### Run the tests on your hardware

```bash
# If you want to use an IP other than 172.16.0.1
colcon build --packages-select franka_bringup --cmake-args -DROBOT_IP=<robot-ip> -DBUILD_TESTING=ON
colcon test --packages-select franka_bringup --event-handlers console_direct+
# to inspect the results
colcon test-result --all --verbose
```

## Surface-Contact Example Profile (Conservative)

For initial contact-force experiments on flat surfaces, use the conservative profile:

```bash
ros2 launch franka_bringup example.launch.py controller_names:=joint_impedance_force_z_down_surface_controller
```

This profile configures the controller with reduced stiffness/torque limits and lower admittance speed
for safer first tests. Parameters are defined in `config/controllers.yaml` under
`joint_impedance_force_z_down_surface_controller`.

### Runtime status topic

The controller publishes runtime tuning/safety status as `std_msgs/msg/Float64MultiArray` on:

```bash
/joint_impedance_force_z_down/surface_status
```

You can inspect it with:

```bash
ros2 topic echo /joint_impedance_force_z_down/surface_status
```
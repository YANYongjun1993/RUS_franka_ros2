# Franka Gazebo

Gazebo visualization support retained for the Franka robot model. The removed
example-controller and mobile-controller launch files are no longer part of
this package.

Launch RViz and Gazebo with:

```bash
ros2 launch franka_gazebo_bringup visualize_franka_robot.launch.py
```

Select another robot type with the `robot_type` argument, or enable the hand
with `load_gripper:=true franka_hand:=franka_hand`.

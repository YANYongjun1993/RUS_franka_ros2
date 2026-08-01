Franka Gazebo
=============

This package retains Gazebo visualization support for the Franka robot model.
The former example-controller and mobile-controller launch files have been
removed.

.. code-block:: shell

    ros2 launch franka_gazebo_bringup visualize_franka_robot.launch.py

Use ``robot_type`` to select a different robot model. The visualization is
configured without a hand, matching the flange-mounted ultrasound probe.

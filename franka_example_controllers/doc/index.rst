Ultrasound Scanning Impedance Controller
========================================

The ``franka_example_controllers`` package contains the Cartesian impedance
controller used for robotic ultrasound scanning with a Franka FR3.

The controller regulates contact force along flange Z, permits hand-guided
motion in flange X/Y, allows compliant roll, and constrains pitch and yaw.

Launch
------

Configure the robot address and namespace in
``franka_bringup/config/franka.config.yaml``, then run:

.. code-block:: shell

    ros2 launch franka_bringup ultrasound_scanning_impedance_controller.launch.py

Configuration
-------------

Controller-manager registration and all runtime parameters are stored in
``franka_bringup/config/controllers.yaml``. The plugin name is:

.. code-block:: text

    franka_example_controllers/UltrasoundScanningImpedanceController

The controller publishes a 24-element runtime status array on
``ultrasound_scanning_impedance/status`` by default.

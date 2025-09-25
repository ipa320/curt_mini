.. _motor_control:

#############
Motor Control
#############

The motors on CURTmini are of type MD80 from MAB robotics.

+-------+----------+
| Motor | Drive ID |
+=======+==========+
| Right | 100      |
+-------+----------+
| Left  | 101      |
+-------+----------+
| Right | 102      |
+-------+----------+
| Left  | 103      |
+-------+----------+

A `fixed version of the candle_ros2 package`_ for ROS 2 Jazzy is provided and already installed.
Motor control is handled through `ros2_control`_.
A `hardware interface`_ is provided for CURTmini, bridging ros2_control to candle_ros2.

.. _`ros2_control`: https://control.ros.org/jazzy/index.html
..
   TODO: publish fixed candle_ros2
.. _`fixed version of the candle_ros2 package`: https://domain.invalid
.. _`hardware interface`: https://github.com/ipa320/curt_mini/tree/main/ipa_ros2_control

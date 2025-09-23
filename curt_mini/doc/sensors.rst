#######
Sensors
#######

IMU
***

The builtin IMU is the model `LPMS-CANAL3`_ from LP-Research.
By default, it is configured to output gyroscope and accelerometer measurements as well as estimated orientation, not using the magnetometer.
The robot base launch file starts the appropriate sensor driver and publishes data on the :code:`/imu/data` topic.

.. _`LPMS-CANAL3`: https://www.lp-research.com/products/inertial-measurement-units-imu/lpms-al3-9-axis-imu-sensor/

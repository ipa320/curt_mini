###########
Electronics
###########

*******
Battery
*******

The battery in CURTmini is a 30000 mAh 6-cell lithium polymer battery pack.
CURTmini or the battery don't include an integrated BMS, it is recommended to use a battery warner device on the battery whenever it is connected to CURTmini, to prevent overdischarge.
The battery input connector is of XT90 type and located in the front left inside the robot.
The battery input is fused with a resettable fuse.

For charging, any 6-cell LiPo charger with integrated balancing will work, as is commonly used and available from suppliers for RC model building.
Specifically, we use the models "Junsi iCharger 406DUO" and "308DUO", with an external 1200W 24V power supply.

.. caution::
   Make sure the charger is configured correctly! Set the cell chemistry to "LiPo", the appropriate battery capacity, the number of cells to 6, and always choose a charging mode which includes cell balancing!

******
Switch
******

The power switch is located on the left back side of the robot.
The latching pushbutton switches the battery connection to the motors and electronics via a relay.
The PC turns on automatically as soon as it is supplied with power.

********
Voltages
********

+---------------+--------+----------------------------------+-----------+-------------------------+
| Voltage       | Color  | Converter                        | Max Power | Consumers               |
+===============+========+==================================+===========+=========================+
| 24V (Battery) | Orange |                                  | 7.9kW     | Motors, DCDC Converters |
+---------------+--------+----------------------------------+-----------+-------------------------+
| 19V           | Green  | QUINT-PS/24DC/24DC/10 (from 24V) | 190W      | NUC (<150W)             |
+---------------+--------+----------------------------------+-----------+-------------------------+
| 12V           | Blue   | RSD-60G-12                       | 60W       | LEDs (5W)               |
+---------------+--------+----------------------------------+-----------+-------------------------+
| Logic Signals | Gray   |                                  |           |                         |
+---------------+--------+----------------------------------+-----------+-------------------------+
| Ground        | Black  |                                  |           |                         |
+---------------+--------+----------------------------------+-----------+-------------------------+

##############
Software Setup
##############

On CURTmini, ROS 2 Jazzy is already installed and a workspace is created, containing the robot base software (this package).

*****************
PC Setup, Network
*****************

The default username to access the robot is :code:`curt`, with password :code:`curtmini`.
You can either attach keyboard and mouse and a screen to the integrated computer (NUC), connect to the robot hotspot :code:`nuc_curt_mini` using password :code:`curtmini`, or manually connect CURTmini to an existing WiFi network.
In the robot hotspot, the NUC has the IP address :code:`10.42.0.1`.

*************
ROS Workspace
*************

A ROS workspace is already set up at `/home/curt/workspace`.

*********
Autostart
*********

When turning on CURTmini, a `tmux`_ session called :code:`nav` is started automatically as specified in the `tmuxp`_ config file in this repo.
By default, the robot base launchfile is started, allowing access to all the integrated sensors, receiving twist commands from navigation software and controlling the robot manually with the joystick.
To customize startup behavior, adjust the tmuxp configuration accordingly.

.. _`tmux`: https://github.com/tmux/tmux/wiki
.. _`tmuxp`: https://github.com/tmux-python/tmuxp

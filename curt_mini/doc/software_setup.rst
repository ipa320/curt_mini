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

Attach to the running TMUX session using:

.. code-block:: console

    $ tmux attach -t nav

.. _`tmux`: https://github.com/tmux/tmux/wiki
.. _`tmuxp`: https://github.com/tmux-python/tmuxp

This is implemented using a systemd service :code:`ipa-ros-autostart`:

.. code-block:: ini

    [Unit]
    Description=CURTmini ROS autostart with tmuxp
    PartOf=ipa-tmux-master.service
    After=ipa-tmux-master.service

    [Service]
    Type=oneshot
    RemainAfterExit=yes
    User=curt
    ExecStart=/usr/bin/tmuxp load -d /home/curt/workspace/src/curt_mini/curt_mini/bringup/autostart.tmuxp.yaml
    ExecStop=/usr/bin/tmux kill-session -t nav

    [Install]
    WantedBy=multi-user.target

Which requires an additional systemd service to start the host tmux session:

.. code-block:: ini

    [Unit]
    Description=tmux master service

    [Service]
    Type=forking
    User=curt
    ExecStart=/usr/bin/tmux new-session -s master -d
    ExecStop=/usr/bin/tmux kill-session -t master

    [Install]
    WantedBy=multi-user.target

****
BIOS
****
The NUC is configured to automatically boot once it is powered, by setting the "After Power Failure" setting to "Power On" in the "Power" menu.
Additionally, the "Fan Mode" setting in the "Cooling" menu is set to "Performance".

***************
Colcon Defaults
***************

A :code:`~/.colcon/defaults.yaml` file is already installed to ensure the robot base packages are built in release mode, and using symlink-install:

.. code:: yaml

    {
      "build": {
        "symlink-install": true,
        "cmake-args": [
          "-DCMAKE_EXPORT_COMPILE_COMMANDS=True",
          "-DCMAKE_BUILD_TYPE=RelWithDebInfo",
        ]
      }
    }

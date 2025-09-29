##############
Software Setup
##############

On CURTmini, ROS 2 Jazzy is already installed and a workspace is created, containing the robot base software (this package).

*****************
PC Setup, Network
*****************

The default username to access the robot is :code:`curt`, with password :code:`curtmini`.
You can either attach keyboard and mouse and a screen to the integrated computer (NUC), connect to the robot hotspot :code:`nuc-curt-mini-...` using password :code:`curtmini`, or manually connect CURTmini to an existing WiFi network.
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

    Wants=multicast-lo.service
    After=multicast-lo.service

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

And a service to enable multicast on the loopback interface, as specified in the `autoware documentation`_.

.. _`autoware documentation`: https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/network-configuration/enable-multicast-for-lo/

***
RMW
***
CycloneDDS is configured as the default RMW implementation using the following environment variables in :code:`.bashrc`:

.. code-block:: bash

  source /opt/ros/jazzy/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="file:///opt/ros/cyclonedds-config.xml"

By default, only the loopback interface is enabled.
Add more network interfaces here if multi-host communication is desired.

.. code-block:: xml

  <CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain Id="any">
      <General>
        <Interfaces>
          <NetworkInterface autodetermine="false" name="lo" priority="default" multicast="default" />
        </Interfaces>
        <AllowMulticast>true</AllowMulticast>
        <MaxMessageSize>65500B</MaxMessageSize>
      </General>
      <Discovery>
        <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
      </Discovery>
      <Internal>
        <SocketReceiveBufferSize min="10MB" />
        <Watermarks>
          <WhcHigh>500kB</WhcHigh>
        </Watermarks>
      </Internal>
    </Domain>
  </CycloneDDS>

The corresponding buffer sizes are specified in :code:`/etc/sysctl.conf`:

.. code-block:: ini

  net.core.rmem_max=2147483647
  net.core.wmem_max=2147483647

****
UDEV
****
Udev rules are installed for permissions and identification of the joystick controller and IMU:

.. code-block:: ini

  # Logitech F710 joystick at /dev/input/f710
  KERNEL=="js[0-9]*", ATTRS{name}=="Logitech Gamepad F710", SYMLINK+="input/f710"

.. code-block:: ini

  # LP-Research IMUs as /dev/tty<SERIAL NUMBER>
  SUBSYSTEM=="tty", ENV{ID_SERIAL_SHORT}=="LPMS*", SYMLINK+="tty%E{ID_SERIAL_SHORT}"

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

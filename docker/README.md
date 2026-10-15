# Curt Mini base robot container

This container builds the local Curt Mini packages and runs
`ros2 launch curt_mini_bringup robot_base.launch.py` on the robot PC. That launch starts
the robot description, Candle motor interface, controllers, joystick teleop,
command mux, and LPMS IMU. It does not start Gazebo, Nav2, Piper, or Ouster.

Copy the entire `curt_mini` repository to the robot PC: the Docker build needs the four package directories, including `curt_mini_bringup/`, as well as `docker/`.


On the robot PC, stop its existing robot bringup first so it does not compete for the motors or joystick. Check that `/dev/bus/usb`, the LPMS serial device, and `/dev/input/f710` exist. Then run:

```bash
cd ~/curt_mini/docker
ROS_DOMAIN_ID=61 docker compose up --build
```

Compose defaults to domain 61. Set a different domain when starting it, such as `ROS_DOMAIN_ID=62 docker compose up --build`. Compose passes that value into the container. 

The entrypoint sources the container's `.bashrc`, which sources ROS Jazzy and the built workspace without changing `ROS_DOMAIN_ID`.

If the robot PC uses different device names, override the host paths:

```bash
LPMS_DEVICE=/dev/ttyUSB0 JOYSTICK_DEVICE=/dev/input/js0 \
  ROS_DOMAIN_ID=61 docker compose up --build
```

To find the host paths:

- IMU
```sh
ls -l /dev/ttyLPMS* /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/* 2>/dev/null
```

To inspect ROS from another terminal on the robot PC:

```bash
cd ~/curt_mini/docker
docker compose exec robot bash -c 'source /root/.bashrc; ros2 control list_controllers'
docker compose exec robot bash -c 'source /root/.bashrc; ros2 topic list'
```

## Controlling the curtmini from your host PC

For RViz on another PC, install and source the matching
`curt_mini_description` package there, as described in `curt_mini_bringup/README.md`.


If you want to teleoperate with your keyboard the curtmini:

```sh
ROS_DOMAIN_ID=61 ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true \
  -p frame_id:=base_link \
  -r cmd_vel:=/cmd_vel
```

# Xbox60 wireless controller with microsoft wireless adapter

The intent with this document is to document the steps taken to get an xbox360 controller (and wireless adapter) working on linux and on the low-level-control docker file.

## Steps taken

- install https://github.com/medusalix/xone to get xbox360 wireless controller to work on linux ubuntu (This should be done on the host computer on which the USB dongle is connected)

- in docker, install ros-humble-joy
  http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick

- js0 should show up when typing ls /dev/input/

- sudo apt-get install jstest-gtk - test joystick functionality - add to dockerfile

- sudo jstest /dev/input/js0 - to test if joystick is recognised - ONLY GOT IT TO WORK WITH PRIVILIGED ATTRIBUTE IN DOCKER-COMPOSE ???

- Add to launch file:
  ros2 joy node can then be started with:
  ros2 run joy joy_node - Assuming the joystick is js0, this can probably be configured as a run argument

- result: echoing /joy displays the xbox360 controller button states and joystick values

- /joy to /cmd_vel topic using ros2/teleop_twist_joy -- https://github.com/ros2/teleop_twist_joy/tree/rolling/

- NOTE: the X button needs to be pressed down whilst moving the joysticks for the cmd_vel topic to be published onto

- Add to dockerfile:
  sudo apt-get install ros-humble-teleop-twist-joy

- Add to launch file:
  ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'

**launch_xbox_controller_feedforward_ctrl.launch.py**

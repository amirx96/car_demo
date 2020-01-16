# Autonomy Example added by Amir

## Change log
I've modifed this car demo code to instead use only nvidia-docker2 and use a ".devcontainer" folder allowing instant configuration of the Docker container by using Microsoft VSC. 
This still requires you to be running Ubuntu 18/16 however, due to the restrictions of nvidia-docker2.

I have also added a Purepursuit example algortihm, which was based on on the Python Robotics library. 

![Purepursuit Demo](demo.gif)
# Demo of Prius in ROS/GAZEBO

This is a simulation of a Prius in [gazebo 9](http://gazebosim.org) with sensor data being published using [ROS kinetic](http://wiki.ros.org/kinetic/Installation)
The car's throttle, brake, steering, and gear shifting are controlled by publishing a ROS message.
A ROS node allows driving with a gamepad or joystick.

# Video + Pictures

A video and screenshots of the demo can be seen in this blog post: https://www.osrfoundation.org/simulated-car-demo/

![Prius Image](https://www.osrfoundation.org/wordpress2/wp-content/uploads/2017/06/prius_roundabout_exit.png)

# Requirements

This demo has been tested on Ubuntu Xenial (16.04)

* An X server
* [Docker](https://www.docker.com/get-docker) - I also think you should not get the snap version of docker if running 18.04
* [nvidia-docker2](https://github.com/nvidia/nvidia-docker/wiki/Installation-(version-2.0))
* The current user is a member of the docker group or other group with docker execution rights.
* ~~[rocker](https://github.com/osrf/rocker)~~ not needed anymore
* Microsoft Visual Studio Code with Remote Workspace Library (WSL)

# Recommended

* A joystick
* A joystick driver which creates links to `/dev/input/js0` or `/dev/input/js1`

This has been tested with the Logitech F710 in Xbox mode. If you have a different joystick you may need to adjust the parameters for the very basic joystick_translator node: https://github.com/osrf/car_demo/blob/master/car_demo/nodes/joystick_translator

# Building
Ensure that you have the nvidia-docker2 package installed, and open this folder with a MS VSC. It should automatically prompt you to reopen this folder in a .devcontainer

# Running
Using a terminal inside the docker container (MS VSC should allow you to create these easy):

```
roslaunch car_demo car_demo.launch
roslaunch wp_control lateralControl.launch
```

If something complains about not being able to connect to a display, run `xhost +local:` on the local machine terminal. This may cause security concerns so this should not be used in a production machine. 


An [RVIZ](http://wiki.ros.org/rviz) window will open showing the car and sensor output.
A gazebo window will appear showing the simulation.
Either use the controller to drive the prius around the world, or click on the gazebo window and use the `WASD` keys to drive the car.

If using a Logitech F710 controller:

* Make sure the MODE status light is off
* Set the swtich to XInput mode
* The right stick controls throttle and brake
* The left stick controls steering
* Y puts the car into DRIVE
* A puts the car into REVERSE
* B puts the car into NEUTRAL

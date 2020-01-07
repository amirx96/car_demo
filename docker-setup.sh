#!/bin/bash
echo "Hello world"
rosdep install -y -r --from-path /prius_ws/src/autonomy
cd /prius_ws && source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make
echo 'source /prius_ws/devel/setup.bash' >> /root/.bashrc

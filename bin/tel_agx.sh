#!/bin/bash
# Program:
#       This program runs for TEL frisbee competition (AGX).
# History:
# 2023/12/07    Betty Lin       First release
PATH=/bin:/sbin:/usr/bin:/usr/sbin:/usr/local/bin:/usr/local/sbin:~/bin
export PATH
echo -e "Happy Cat ヽ(=^･ω･^=)丿- AGX \a \n"

cd ~/ros2_control_arduino/
  
# Source environments
source /opt/ros/foxy/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source install/setup.bash

# Run
#echo -e "Looking for ports... \a \n"
#ros2 run control find_ports
ros2 run control arduino_subscriber
  
exit 0


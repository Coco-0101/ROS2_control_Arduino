# Control Arduino in ROS 2

## Preparing
### Permission denied
If encounter with an error like `Permission denied: '/dev/ttyACM0'`  
Type this command to terminal:

    sudo chmod a+rw /dev/ttyACM0

### Make Arduino not need to use sudo
From this example, the device name that Arduino corresponds to the system is `ttyACM0`.
```
$ ls -l /dev/ttyACM0
crw-rw---- 1 root dialout 166, 0 Sep 20 15:40 /dev/ttyACM0
```
Note that its permissions are only given to the dialout group to read and write.

In order to be able to read and write the device `/dev/ttyACM0`, you must add all accounts that want to use the Arduino IDE to the dialout group, and then log in again.

    sudo usermod -G dialout -a <USER>

e.g.

    sudo usermod -G dialout -a betty

### Arduino Code
Upload code(arduino/XXX.ino) to Arduino board.

### Build with colcon

    colcon build --symlink-install

### Source the setup.bash file

    source install/setup.bash

## Start controlling
### Using `find_ports` for find correct port
Find correct serial port:

    ros2 run control find_ports

### Send signal to Arduino
To run these code you need to know correct port path, baud rate and if you want to use serial writing you need to know topic name to subscribe.

    ros2 run control <node_name> --ros-args -p serial_port:=<device_port> -p baud_rate:=<baud_rate> -p subscribe_to:=<topic_name>

#### Using `keyboard` to publish message from keyboard

    ros2 run control keyboard --ros-args -p serial_port:=/dev/ttyACM0  -p baud_rate:=115200 -p subscribe_to:=arduino_command

#### Using `remote_controller` to publish message from remote controller

    ros2 run control remote_controller --ros-args -p serial_port:=/dev/ttyACM0  -p baud_rate:=115200 -p subscribe_to:=arduino_command

#### Without parameters
##### Default setting
- device_port : /dev/ttyACM0
- baud_rate : 115200
- subscribe to arduino_command
###### keyboard
    ros2 run control keyboard
###### remote controller
    ros2 run control remote_controller

>Note: Baud rate of the arduino/XXX.ino and the node need to be same.

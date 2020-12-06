M5Bot Firmware
==============

# Verified Environment

* M5GO
* macOS Catalina 10.15.7
  * Please Install Following.
    * Arduino IDE
    * Docker

# Installation

## M5Stack

1. Create wifi_setting.h with reference to wifi_setting.h.template
   1. copy wifi_setting.h.template wifi_setting.h
   2. edit wifi_setting.h
2. Edit ROS_MASTER_URI in the sketch.
3. Upload Sketch with Arduino IDE

## Mac

```
$ cd m5bot_firm
$ docker-compose build
```

# Teleop with Keyboard using Docker Container

Startup containers and launch teleop_twist_keyboard package.

```
$ docker-compose up
$ docker ps # Check your container id
$ docker exec -it $BRIDGE_CONTAINER_ID /bin/bash -c 'source /opt/ros/melodic/setup.bash && rosrun teleop_twist_keyboard teleop_twist_keyboard.py'
```

After that please turn on M5Go and enjoy teleop!

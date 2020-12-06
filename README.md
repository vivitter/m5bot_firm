M5Bot Firmware
==============

This is a ROS Node running on M5Go.
You can control M5Bot via Wi-Fi.

![M5Bot](https://user-images.githubusercontent.com/1230152/101273091-eb4a2a00-37d5-11eb-92f1-e643a18eb720.jpg)

# Verified Environment

* M5GO
* macOS Catalina 10.15.7
  * Arduino IDE
    * You need to install following libraries.
      * m5stack
      * rosserial
  * Docker


# ROS Topic

* Subscribing Topics
  * /cmd_vel
* Publishing Topics
  * /imu:

# Installation

## M5Stack

1. Create wifi_setting.h with reference to wifi_setting.h.template
   1. copy wifi_setting.h.template wifi_setting.h
   2. edit wifi_setting.h
2. Upload Sketch with Arduino IDE

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

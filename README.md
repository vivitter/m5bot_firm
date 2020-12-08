M5Bot Firmware
==============

This is a ROS Node running on M5Go.
You can control M5Bot via Wi-Fi.

![M5Bot](https://user-images.githubusercontent.com/1230152/101273091-eb4a2a00-37d5-11eb-92f1-e643a18eb720.jpg)

# Verified Environment

* Hardware
  * M5Go
  * M5Stack Modules
    * [Servo Module Board 12 Channels](https://m5stack.com/collections/m5-module/products/servo-module)
  * Rotation Servo Motor
    * [GeeKservo BP3005-02 JM9G360](https://nettigo.eu/products/geekservo-lego-compatible-continuous-rotation-servo-360)
  * Lego Block
  * [Mecanum Wheel](https://www.switch-science.com/catalog/5939/)
* macOS Catalina 10.15.7
  * Arduino IDE
    * You need to install following libraries.
      * m5stack
      * rosserial
  * Docker


# ROS Topic

* Subscribing Topics
  * /m5bot/cmd_vel
* Publishing Topics
  * /m5bot/imu:

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
$ docker exec -it $BRIDGE_CONTAINER_ID /bin/bash -c 'source /opt/ros/melodic/setup.bash && rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/m5bot/cmd_vel'
```

After that please turn on M5Go and enjoy teleop!

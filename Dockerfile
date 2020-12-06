FROM ros:melodic
RUN apt update && \
    apt install -y ros-melodic-rosserial && \
    apt install -y ros-melodic-rosserial-arduino && \
    apt install -y ros-melodic-teleop-twist-keyboard
# Base image
FROM ros:humble

ENV TZ=Europe/Amsterdam
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update && apt-get install kodi-peripheral-joystick -y

# Basic tools
# RUN apt-get update
# RUN apt-get update && \
#     apt-get install vim nano git tmux wget curl python-pip net-tools iputils-ping  -y

# Install additional ros packages
# RUN apt-get update && apt-get install ros-melodic-rosbridge-server ros-melodic-joy -y
# RUN pip install adafruit-pca9685

# Add ros sourcing to bashrc
RUN echo ". /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create Ros workspace
ENV EXOMY_WS=/WS/orion_ws
RUN mkdir -p $EXOMY_WS/src

WORKDIR /WS/orion_ws

COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]

FROM osrf/ros:melodic-desktop-full
RUN apt update && apt install -y git wget
ENV DISPLAY=host.docker.internal:0.0

### Add User ID and Group ID
ARG UNAME=plt
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID -o $UNAME
RUN useradd -m -u $UID -g $GID -o -s /bin/bash $UNAME

ARG CODE_DIR=/home/${UNAME}

### ROS setup ###
RUN mkdir -p ${CODE_DIR}/catkin_ws/src && \
    echo "source /opt/ros/melodic/setup.bash" >> ${CODE_DIR}/.bashrc

#### Install dependencies ###
RUN apt-get update && apt-get install -y python-catkin-tools

### Install Power line tracker with event camera
RUN cd ${CODE_DIR}/catkin_ws/src && \
    git clone https://github.com/mikihiroikura/line_tracking_with_event_cameras.git && \
    git clone https://github.com/uzh-rpg/rpg_dvs_ros.git && \
    git clone https://github.com/catkin/catkin_simple.git

### 
RUN chown -R $UNAME:$UNAME ${CODE_DIR}/catkin_ws

USER $UNAME
WORKDIR ${CODE_DIR}
FROM osrf/ros:melodic-desktop-full
RUN apt update && apt install -y git wget
ENV DISPLAY=host.docker.internal:0.0

### Add User ID and Group ID
ARG UNAME=plt
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID -o $UNAME
RUN useradd -m -u $UID -g $GID -o -s /bin/bash $UNAME

# Add User into sudoers, can run sudo command without password
RUN apt update && apt install -y sudo
RUN usermod -aG sudo ${UNAME}
RUN echo "${UNAME} ALL=(ALL) NOPASSWD:ALL" | tee /etc/sudoers.d/${UNAME}

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

### Change owner
RUN chown -R $UNAME:$UNAME ${CODE_DIR}/catkin_ws

### Switch user
USER $UNAME
WORKDIR ${CODE_DIR}

### ROS setup for catkin_ws
RUN cd ${CODE_DIR}/catkin_ws && \
    catkin config --init --mkdirs --extend /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build line_event_tracker && \
    catkin build line_event_tracker_visualizer && \
    catkin build dvs_renderer && \
    echo "source /home/${UNAME}/catkin_ws/devel/setup.bash" >> /home/${UNAME}/.bashrc

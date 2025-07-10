FROM osrf/ros:noetic-desktop-full

ENV DISPLAY=host.docker.internal:0.0

# Update ROS gpg key (because gpg keys was expired on June 2025)
ADD https://raw.githubusercontent.com/ros/rosdistro/master/ros.key /tmp/ros.key
RUN rm -rf /etc/apt/sources.list.d/ros1-latest.list && \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg /tmp/ros.key && \
    rm /tmp/ros.key && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $( . /etc/os-release && echo $UBUNTU_CODENAME ) main" > /etc/apt/sources.list.d/ros1-latest.list
RUN apt update && apt install -y git wget

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
    echo "source /opt/ros/noetic/setup.bash" >> ${CODE_DIR}/.bashrc

#### Install dependencies ###
RUN apt-get update && apt-get install -y python3-catkin-tools

### Install Power line tracker with event camera
COPY . ${CODE_DIR}/catkin_ws/src

### Change owner
RUN chown -R $UNAME:$UNAME ${CODE_DIR}/catkin_ws

### Switch user
USER $UNAME
WORKDIR ${CODE_DIR}

### ROS setup for catkin_ws
RUN cd ${CODE_DIR}/catkin_ws && \
    catkin config --init --mkdirs --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build line_event_tracker && \
    catkin build line_event_tracker_visualizer && \
    catkin build dvs_renderer && \
    echo "source /home/${UNAME}/catkin_ws/devel/setup.bash" >> /home/${UNAME}/.bashrc

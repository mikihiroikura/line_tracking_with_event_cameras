# Line Tracking with Event Cameras
This is a forked repository from [Power Line Tracking with Event Cameras](https://github.com/uzh-rpg/line_tracking_with_event_cameras).<br>
This repository is used for evaluation of event-based line segment detection and tracking.

## Installation
Build docker image by the following command:
```
docker build --build-arg UID=$(id -u) --build-arg GID=$(id -g)  -t plt:latest .
```

## Run
Run tracker and record results
```
cd ~/catkin_ws
roslaunch line_event_tracker run_with_visualizer_record.launch bag_file:=/app/rosbag/ICCV2025/log2bag/three_vertical_lines_fast.bag bag_start_time:=1738661335.90 play_speed:=0.05
```
- `bag_file`: Select a bagfile for rosbag play
- `bag_start_time`: Check the start time of rosbag by the following command `rosbag info /your/rosbag/path`
- `play_speed`: Specify rosbag play speed.
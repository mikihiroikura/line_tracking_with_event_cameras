#!/bin/bash

DATA_PATH=${1:-/app/rosbag/ICCV2025/MVSEC/indoor_flying1_data.bag}
OUTPUT_UNDISTORT_DIR=${2:-~/data/LineSegments/ICCV2025/rosbag_mvsec_indoor1/powerline_undistort/r005_paramrate10}
BAG_START_TIME=${3:-1504645177.42}
PLAY_SPEED=${4:-0.2}
EVENT_TOPIC=${5:-/davis/left/events}
IMAGE_TOPIC=${6:-/davis/left/image_raw}
START_TIME=${7:-15}
START_TRIAL=${8:-1}
END_TRIAL=${9:-30}

for i in $(seq $START_TRIAL $END_TRIAL); do
    docker exec plt /bin/bash -c "cd ~/catkin_ws && source devel/setup.bash && roslaunch line_event_tracker run_with_visualizer_record.launch bag_file:=$DATA_PATH bag_start_time:=$BAG_START_TIME play_speed:=$PLAY_SPEED event_topic:=$EVENT_TOPIC image_topic:=$IMAGE_TOPIC start_time:=$START_TIME"

    docker cp plt:/home/plt/lines.txt "$OUTPUT_UNDISTORT_DIR/$i.txt"

done



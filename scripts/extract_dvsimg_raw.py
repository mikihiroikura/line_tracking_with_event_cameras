#!/usr/bin/env python2.7.17

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Video setting
video_filename = "output_video.mp4"  # File name
frame_width = 346                   # Frame width
frame_height = 260                  # Frame height

# VideoWriter
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Video format

# Records
cv_images = []
timestamp_init = None
timestamp_end = None

# CvBridge Initialization
bridge = CvBridge()

def image_callback(msg):
    global cv_images
    global timestamp_end, timestamp_init
    try:
        # Convert from rosbag format to cv2
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Add frame
        cv_images.append(cv_image)
        
        # Update timestamp
        if timestamp_init is None:
            timestamp_init = msg.header.stamp.to_sec()
        timestamp_end = msg.header.stamp.to_sec()

    except Exception as e:
        rospy.logerr('Failed to process image: {}'.format(e))

if __name__ == "__main__":
    rospy.init_node("image_to_video_saver")

    # Define topic name
    topic_name = "/dvs/image_raw"
    rospy.Subscriber(topic_name, Image, image_callback)

    rospy.loginfo('Listening to {}... Press Ctrl+C to stop.'.format(topic_name))

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down.")
        
    # Create video
    fps = len(cv_images) / (timestamp_end - timestamp_init)
    video_writer = cv2.VideoWriter(video_filename, fourcc, fps, (frame_width, frame_height))
    for cv_image in cv_images:
        video_writer.write(cv_image)

    # Relase
    video_writer.release()
    cv2.destroyAllWindows()
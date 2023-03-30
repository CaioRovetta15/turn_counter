#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time
import os 


def publish_video():
    # Create a node to publish the video
    rospy.init_node('video_publisher')
    input_video = rospy.get_param('/video_publisher/video_path') # Path to the video file

    # Create a publisher for the ROS image message
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)

    # Initialize the CvBridge object
    bridge = CvBridge()

    # Open the video file
    cap = cv2.VideoCapture(input_video)
    # Loop through the frames of the video
    while not rospy.is_shutdown() and cap.isOpened():
        # Read a frame from the video
        ret, frame = cap.read()
        #display the frame 
        if ret:
            try:
                # Convert the OpenCV image to a ROS image message
                ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
            except CvBridgeError as e:
                print(e)
            else:
                ros_image.header.stamp = rospy.Time.from_sec(time.time())
                # Publish the ROS image message to the topic
                image_pub.publish(ros_image)
        else:
            break
        # Wait for a small amount of time before processing the next frame
        rospy.sleep(1.0/15.0) # 15 fps
    # Release the video file and shutdown the node
    cap.release()
    cv2.destroyAllWindows()
    rospy.signal_shutdown("Video publishing finished")
    os.system("rosnode kill /orb_slam3 /orb_slam3_ros/trajectory_server_orb_slam3 /rviz")

if __name__ == '__main__':
    publish_video()

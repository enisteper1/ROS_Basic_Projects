#!/usr/bin/env python3

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys


def main():
    rospy.init_node("image_pub_node", anonymous=True)
    image_pub = rospy.Publisher("/camera/image", Image, queue_size=10)
    bridge = CvBridge()
    cap = cv2.VideoCapture("/home/enis/ROS_Basic_Projects/src/ros_projects/src/resources/tennis-ball-video.mp4")
    #cap = cv2.VideoCapture(0)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break
        image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
        rate.sleep()

if __name__ == "__main__":
    main()


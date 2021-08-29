#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class SubNode_Ball_Detector:

    def __init__(self, topic):
        self.yellow_lower_bound = (30, 150, 50)
        self.yellow_upper_bound = (60, 255, 255)
        self.bridge = CvBridge()
        
        self.source_image = None
        self.masked_image = None
        self.contours = None
        
        self.image_subscriber = rospy.Subscriber(topic, Image, self.callback)

    def process_image(self):
        image_hsv = cv2.cvtColor(self.source_image, cv2.COLOR_BGR2HSV)
        self.masked_image = cv2.inRange(image_hsv, self.yellow_lower_bound, self.yellow_upper_bound)
        
        self.get_contours()

    def get_contours(self):
        self.contours, hierarchy = cv2.findContours(self.masked_image.copy(),
                                             cv2.RETR_EXTERNAL,
                                             cv2.CHAIN_APPROX_SIMPLE)
        self.draw_contours()

    def draw_contours(self):
        empty_img = np.zeros_like(self.masked_image, dtype=np.uint8)
        for contour in self.contours:
            area = cv2.contourArea(contour)
            if area > 3000:
                ((x, y), radius) = cv2.minEnclosingCircle(contour)
                center_x, center_y = self.get_contour_center(contour)
                cv2.circle(self.masked_image, (center_x, center_y), int(radius), (0, 0, 255), 1)
                cv2.circle(self.source_image, (center_x, center_y), int(radius), (0, 0, 255), 1)
        
        cv2.imshow("Processed Video", self.masked_image)
        cv2.imshow("Final Image", self.source_image)
        cv2.waitKey(1)


    def get_contour_center(self, contour):
        moments = cv2.moments(contour)
        cx = -1
        cy = -1
        if moments["m00"] != 0:
            cx = int(moments["m10"]/moments["m00"])
            cy = int(moments["m01"]/moments["m00"])

        return cx, cy
        
    def callback(self, msg):
        try:
            self.source_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_image()
        except CvBridgeError as e:
            print(e)

def main():
    topic = "/camera/image"
    sub_ball_det = SubNode_Ball_Detector(topic)

    rospy.init_node("ball_detector", anonymous=True)
    try:
        rospy.spin()
    except:
        cv2.destroyAllWnidows()

if __name__ == "__main__":
    main()


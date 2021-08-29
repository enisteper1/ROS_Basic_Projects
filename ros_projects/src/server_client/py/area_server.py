#!/usr/bin/env python3
import rospy
from ros_projects.srv import RectangleArea
from ros_projects.srv import RectangleAreaRequest
from ros_projects.srv import RectangleAreaResponse
import time

def calculate_area(req):
    print("Area of width: %s height: %s is %s " %(req.width, req.height, (req.width * req.height)))
    time.sleep(0.2)
    response = RectangleAreaResponse(req.width * req.height)
    return response

if __name__ == "__main__":
    # Init node
    rospy.init_node("calculate_area_server")
    # Init service as calculate_area
    service = rospy.Service("calculate_area", RectangleArea, calculate_area)
    print("Ready to calculate the area")
    # Infinite loop
    rospy.spin()

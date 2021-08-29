#!/usr/bin/env python3
import rospy
import sys
from ros_projects.srv import RectangleArea
from ros_projects.srv import RectangleAreaRequest
from ros_projects.srv import RectangleAreaRequest

def calculate_area_client(width, height):
    rospy.wait_for_service("calculate_area")
    try:
        fnc = rospy.ServiceProxy("calculate_area", RectangleArea)
        response = fnc(width, height)
    except rospy.ServiceException as e:
        print("Could not call the service" %e)
    return response

if __name__ == "__main__":
    if len(sys.argv) == 3:
        width = float(sys.argv[1])
        height = float(sys.argv[2])
    else:
        print("Usage area_client.py width height")
        sys.exit(1)
    print("Area of  %s * %s is %s"%(width, height, calculate_area_client(width, height)))

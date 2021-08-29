#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math
import time

def rotate (angular_speed_degree, relative_angle_degree, clockwise):
    
    velocity_message = Twist()
    velocity_message.linear.x=0
    velocity_message.linear.y=0
    velocity_message.linear.z=0
    velocity_message.angular.x=0
    velocity_message.angular.y=0
    velocity_message.angular.z=0
 
    theta0=0
    angular_speed=math.radians(angular_speed_degree)

    if clockwise:
        velocity_message.angular.z =-abs(angular_speed)
    else:
        velocity_message.angular.z =abs(angular_speed)

    angle_moved = 0.0
    loop_rate = rospy.Rate(100) # we publish the velocity at 10 Hz (10 times a second)    
    cmd_vel_topic='/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    t0 = rospy.Time.now().to_sec()
    while True:
        try:
            rospy.loginfo("Turtlesim rotates")
            velocity_publisher.publish(velocity_message)

            t1 = rospy.Time.now().to_sec()
            current_angle_degree = (t1-t0)*angular_speed_degree
            loop_rate.sleep()
           
            if  (current_angle_degree>math.radians(relative_angle_degree)):
                rospy.loginfo("reached")
                break
        except rospy.ROSInterruptException:
            break

    #finally, stop the robot when the distance is moved
    velocity_message.angular.z =0
    velocity_publisher.publish(velocity_message)

def setDesiredOrientation(desired_angle_radians):
    relative_angle_radians = desired_angle_radians
    if relative_angle_radians < 0:
        clockwise = 1
    else:
        clockwise = 0
    rotate(30 ,math.degrees(abs(relative_angle_radians)), clockwise)

if __name__ == "__main__":
    rospy.init_node("rotate_node", anonymous=True)
    #rotate(30, 90, True)
    setDesiredOrientation(90)

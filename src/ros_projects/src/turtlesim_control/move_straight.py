#!/usr/bin/env python3
import rospy
import time
from geometry_msgs.msg import Twist

def move(velocity_publisher, speed, distance, isForward):
        velocity_message = Twist()
        if isForward:
            velocity_message.linear.x = abs(speed)
        else:
            velocity_message.linear.x  = - abs(speed)
        velocity_message.linear.y = 0 
        velocity_message.linear.z = 0

        velocity_message.angular.x = 0
        velocity_message.angular.y = 0
        velocity_message.angular.z = 0       
        
        t0 = rospy.get_time()
        current_distance = 0.0
        loop_rate = rospy.Rate(100)
        while current_distance < distance:
            velocity_publisher.publish(velocity_message)
            t1 = rospy.get_time()
            current_distance = speed * (t1 - t0)
            loop_rate.sleep()

        velocity_message.linear.x = 0
        velocity_publisher.publish(velocity_message)

if __name__ == "__main__":
    rospy.init_node("move_straight", anonymous=True)
    velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    move(velocity_publisher, 2.0, 4.0, False)
    #move(velocity_publisher, 2.0, 4.0, True)


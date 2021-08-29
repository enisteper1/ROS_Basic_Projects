#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
import math

x = y = yaw = 0
def pose_callback(msg):
    global x, y, yaw
    x, y, yaw = msg.x, msg.y, msg.theta


def move2goal( goal_pose, distance_tolerance):
    global x, y, yaw
    velocity_message = Twist()
    loop_rate = rospy.Rate(100)
    pub_topic = "/turtle1/cmd_vel"
    velocity_publisher = rospy.Publisher(pub_topic, Twist, queue_size=10)

    Kp1 = 1.0
    Kp2 = 4.0
    current_distance = 100.0
    
    while current_distance > distance_tolerance:
        current_distance = getDistance(x, y, goal_pose.x, goal_pose.y)
        velocity_message.linear.x = Kp1 * current_distance
        velocity_message.angular.z = Kp2 *(math.atan2(goal_pose.y - y, goal_pose.x - x) - yaw)
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()

    velocity_message.linear.x = 0
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)

def getDistance(x1, y1, x2, y2):
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

if __name__ == "__main__":
    rospy.init_node("go2goal_node", anonymous=True)
    pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, pose_callback)
    pose = Pose()
    pose.x = 2
    pose.y = 2
    pose.theta = 0
    move2goal(pose, 0.1)

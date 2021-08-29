#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time

class TurtleBot:
    def __init__(self):
        
        self.velocity_publisher = None
        self.subscriber = None
        self.loop_rate = rospy.Rate(100)
        
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.x_min = 0.0
        self.x_max = 11.0
        self.y_min = 0.0
        self.y_max = 11.0

    def poseCallback(self, pose_message):
        self.x = pose_message.x 
        self.y = pose_message.y
        self.yaw = pose_message.theta
    
    def move(self, speed, distance, isForward):
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

        while current_distance < distance:
            #rospy.loginfo("Moving forward")
            self.velocity_publisher.publish(velocity_message)
            self.loop_rate.sleep()
            t1 = rospy.get_time()
            current_distance = speed * (t1 - t0)

        velocity_message.linear.x = 0
        self.velocity_publisher.publish(velocity_message)
    
    def rotate(self, angular_speed, relative_angle, clockwise):
        angular_speed = self.degrees2radians(angular_speed)
        relative_angle = self.degrees2radians(relative_angle)
        velocity_message = Twist()
        velocity_message.linear.x = 0
        velocity_message.linear.y = 0 
        velocity_message.linear.z = 0

        velocity_message.angular.x = 0
        velocity_message.angular.y = 0
        if clockwise:
            velocity_message.angular.z = abs(angular_speed)
        else:
            velocity_message.angular.z = -abs(angular_speed)
        
        current_angle = 0
        t0 = rospy.get_time()
        while current_angle < relative_angle:
            self.velocity_publisher.publish(velocity_message)
            t1 = rospy.get_time()
            current_angle = angular_speed * (t1 - t0)
            self.loop_rate.sleep()

        velocity_message.angular.z = 0
        self.velocity_publisher.publish(velocity_message)

    def getDistance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def degrees2radians(self, angle_in_degrees):
        return angle_in_degrees * math.pi / 180.0
    
    def setDesiredOrientation(self, desired_angle_radians):
        desired_angle_radians = self.degrees2radians(desired_angle_radians)
        relative_angle_radians = desired_angle_radians - self.yaw
        if relative_angle_radians < 0:
            clockwise = True
        else:
            clockwise = False

        self.rotate(self.degrees2radians(30), abs(relative_angle_radians), clockwise)

    def moveGoal(self, goal_pose, distance_tolerance):
        velocity_message = Twist()
        Kp1 = 1.0
        Kp2 = 4.0
        current_distance = 10.0
        while current_distance > distance_tolerance:
            current_distance = self.getDistance(self.x, self.y, goal_pose.x, goal_pose.y)
            velocity_message.linear.x = Kp1 * current_distance
            velocity_message.angular.z = Kp2 *(math.atan2(goal_pose.y - self.y, goal_pose.x - self.x) - self.yaw)
            self.velocity_publisher.publish(velocity_message)
            self.loop_rate.sleep()

        velocity_message.linear.x = 0
        velocity_message.angular.z = 0
        self.velocity_publisher.publish(velocity_message)

    def moveXY(self,speed, goal_x, goal_y):
        velocity_message = Twist()
        y_distance = goal_y - self.y
        x_distance = goal_x - self.x
        print(y_distance)
        print(x_distance)
        time.sleep(2)
        if y_distance > 0:
            velocity_message.linear.y = abs(speed)
        else:
            velocity_message.linear.y = -abs(speed)

        if x_distance > 0:
            velocity_message.linear.x = abs(speed)
        else:
            velocity_message.linear.x  = -abs(speed)


        #velocity_message.linear.y = 0 
        velocity_message.linear.z = 0

        velocity_message.angular.x = 0
        velocity_message.angular.y = 0
        velocity_message.angular.z = 0       
        
        current_distance = 10.0
        distance_tolerance = 0.05

        while True:
            try:
                #rospy.loginfo("Moving forward")
                self.velocity_publisher.publish(velocity_message)
                print(goal_x - self.x)
                print(goal_y - self.y)
                self.loop_rate.sleep()
                if abs(goal_y - self.y) < 0.25:
                    velocity_message.linear.y = 0
                if abs(goal_x - self.x) < 0.25:
                    velocity_message.linear.x = 0

                if velocity_message.linear.x == 0 and velocity_message.linear.y == 0:
                    break

                current_distance = self.getDistance(self.x, self.y, goal_x, goal_y)
            except KeyboardInterrupt:
                break


        velocity_message.linear.x = 0
        velocity_message.linear.y = 0
        self.velocity_publisher.publish(velocity_message)
    
    def gridClean(self):
        pose = Pose()
        coordinates = [[5, 1], [1, 1], [1, 9], [5, 9], [5, 1], [9, 1], [9, 9]]

        for coordinate in coordinates:
            self.rotate(30, 90, False)
            self.loop_rate.sleep()
            self.moveXY(2.0, coordinate[0], coordinate[1])


    def spiralClean(self):
        vel_msg = Twist()
        vel_msg.linear.y =0
        vel_msg.linear.z =0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 4
        loop_rate = rospy.Rate(1)
        while((self.x<10.25) and (self.y<10.25)):
            vel_msg.linear.x += 1
            self.velocity_publisher.publish(vel_msg)
            loop_rate.sleep()
    
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

if __name__ == "__main__":
    rospy.init_node("clean_robot", anonymous=True)
    
    turtle_bot = TurtleBot()
    velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    
    pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, turtle_bot.poseCallback)
    turtle_bot.velocity_publisher = velocity_publisher
    turtle_bot.pose_subscriber = pose_subscriber
    time.sleep(2)
    #turtle_bot.move(2.0, 4.0, False) # Move to certain distance at x 
    #turtle_bot.rotate(30, 90, False) # Rotae 90 degree at 80 degree/sec
    #turtle_bot.setDesiredOrientation(90)
    """
    pose = Pose()
    pose.x = 1
    pose.y = 9
    pose.theta = 0
    turtle_bot.moveGoal(pose, 0.05)
    """
    #turtle_bot.gridClean()
    turtle_bot.spiralClean()
    #turtle_bot.moveXY(2, 1, 1)

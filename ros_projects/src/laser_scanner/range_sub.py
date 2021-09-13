#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math



class TurtleController:

    def __init__(self):
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.scan_subscriber = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        self.loop_rate = rospy.Rate(100) # 100 Hz
        self.scan_msg = None
        self.velocity_message = Twist()
        self.min_dist = 0
        self.linear_propotional = 0.4
        self.clockwise = True

    def move_forward(self, speed):
        self.velocity_message.linear.x = speed
        self.velocity_message.angular.z = 0
    
    def move_propotional(self):
        self.velocity_message.angular.z = 0
        speed = self.min_dist * self.linear_propotional
       
       if speed > 0.5:
            self.velocity_message.linear.x = 0.5
        
    def rotate(self, speed):
        self.velocity_message.linear.x = 0
       
       if self.clockwise:
            self.velocity_message.angular.z = abs(math.radians(speed))
        else:
            self.velocity_message.angular.z = -abs(math.radians(speed))
       
       while self.min_dist < 1.25:
          self.velocity_publisher.publish(self.velocity_message)
          self.loop_rate.sleep()
    
    def scan_callback(self, scan_msg):
        self.scan_msg = scan_msg
        
        dist1 = self.distance_at_ranges(340, 360)
        dist2 = self.distance_at_ranges(0, 20)
        
        if dist1 >= dist2:
            self.clockwise = False
        else:
            self.clockwise = True
        self.min_dist = min(dist1, dist2)

    def distance_at_ranges(self, start_index, end_index):
        ranges = [x for x in self.scan_msg.ranges if not math.isnan(x)]
        slice_of_array = ranges[start_index: end_index+1]
        return min(slice_of_array)

    def main_loop(self):
        while not rospy.is_shutdown():
            if self.min_dist < 0.6:
                self.rotate(30)
            else:
                self.move_propotional()
            self.velocity_publisher.publish(self.velocity_message)
            self.loop_rate.sleep()


if __name__ == '__main__':
    
    rospy.init_node('scan_node', anonymous=True)
    turtlebot3_controller = TurtleController()
    turtlebot3_controller.main_loop()
    rospy.spin()

#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback_message(message):
    #get id and print message
    rospy.loginfo(rospy.get_caller_id() + "Recieved Message: %s", message.data)
    

if __name__ == '__main__':
    # Initialize node as subscriber
    rospy.init_node('subscriber', anonymous=True)
    # Define topic which will be listened
    rospy.Subscriber("pub_message", String, callback_message)
    # Infinite loop
    rospy.spin()

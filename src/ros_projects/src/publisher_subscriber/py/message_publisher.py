#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def publisher():
    # In order to publish a topic first of all a node should be initialized
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(5) #  5Hz which means 5 time run at sec
    # Defining a publisher which publishes message to pub_message topic
    message_pub = rospy.Publisher('pub_message', String, queue_size=10)
    cnt = 0
    while not rospy.is_shutdown():
        try:
            message = f"Message Published {cnt}"
            cnt += 1
            rospy.loginfo(message)
            message_pub.publish(message)
            rate.sleep()
        except KeyboardInterrupt:
            break

if __name__ == '__main__':
        publisher()
        

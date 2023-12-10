#!/usr/bin/env python2

import rospy
from std_msgs.msg import String

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def recieve_message():
	rospy.init_node('pixy_python_subscriber', anonymous = True)
	rospy.Subscriber("pixy_data_py", String, callback)
	rospy.spin()

if __name__ == '__main__':
	recieve_message()

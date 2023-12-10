#!/usr/bin/env python2

import rospy
from std_msgs.msg import String
import pixy
from pixy import *

pixy.init()
pixy.change_prog("video")
pixy.set_lamp(1,0)
X = 158
Y = 104
Frame = 1

def publish_message():
    pub = rospy.Publisher('pixy_data_py', String, queue_size=10)
    rospy.init_node('pixy_publisher', anonymous = True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg_str = video_get_RGB(X,Y)
        rospy.loginfo(str(msg_str))
        pub.publish(str(msg_str))
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass

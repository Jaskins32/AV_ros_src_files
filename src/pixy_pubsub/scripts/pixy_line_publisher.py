#!/usr/bin/env python2

from __future__ import print_function
import Jetson.GPIO as GPIO
import pixy
from ctypes import *
from pixy import *
import rospy
from geometry_msgs.msg import Twist
import math


pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)
rospy.init_node('line_follow_publisher', anonymous = True)
rate = rospy.Rate(10)

en = False

def en_interrupt(ENBUTTON):
    global en
    en = True
    rospy.loginfo("bp registered")

GPIO.setmode(GPIO.BOARD)

class Vector (Structure):
  _fields_ = [
    ("m_x0", c_uint),
    ("m_y0", c_uint),
    ("m_x1", c_uint),
    ("m_y1", c_uint),
    ("m_index", c_uint),
    ("m_flags", c_uint)] 
vectors = VectorArray(10)
ENBUTTON = 15
GPIO.setup(ENBUTTON, GPIO.IN)
GPIO.add_event_detect(ENBUTTON, GPIO.RISING, callback = en_interrupt, bouncetime= 30)
X_CENTER = 39
move_cmd = Twist()
valid_line = False
line_found = False
en = False
while en == False:
    move_cmd.linear.x = 0
    move_cmd.angular.z = 0
    pub.publish(move_cmd)
    rate.sleep()
rospy.loginfo("starting program")
GPIO.remove_event_detect(ENBUTTON)
pixy.init()
pixy.change_prog("line")
pixy.set_lamp(1,0)
while not rospy.is_shutdown():
    try:
        #rospy.loginfo("looking for line")
        line_get_main_features()
        i_count = line_get_vectors(10, vectors)
        if i_count <= 0 and line_found == False:
            move_cmd.linear.x = 0
            move_cmd.angular.z = 0
            rospy.loginfo("no line found")
        elif (i_count != 0):
            line_found = True
            #rospy.loginfo("line found")
            angle_radians = math.atan2(vectors[0].m_y1 - vectors[0].m_y0, vectors[0].m_x1 - vectors[0].m_x0)
            angle_degrees = math.degrees(angle_radians)
            rospy.loginfo(angle_degrees)
            if ((angle_degrees < -60 and angle_degrees > -120) or (angle_degrees> 60 and angle_degrees < 120 )):
                #rospy.loginfo("valid line found")
                valid_line = True
            else:
                rospy.loginfo(angle_degrees)
        #if(line_found == True):
            #move_cmd.linear.x = 0.5
        if (valid_line):
            rospy.loginfo(angle_degrees)
            move_cmd.linear.x = 1
            error = vectors[0].m_x1 - X_CENTER
            if(error > 4):
                move_cmd.angular.z = -0.5
            elif(error < -4):
                move_cmd.angular.z = 0.5
            else:
                move_cmd.angular.z = 0
        #rospy.loginfo("reached here")
        pub.publish(move_cmd)
        rate.sleep()
        #rospy.loginfo("reached end of loop")
    except rospy.ROSInterruptException:
            #print("exit with ros interrupt")
            pass
    except KeyboardInterrupt:
        #print("exit with keyboard")
        move_cmd.linear.x = 0
        move_cmd.angular.z = 0
        pub.publish(move_cmd)
        pixy.m_link.close()
        rate.sleep()
        rospy.spin()
    finally:
        GPIO.cleanup()


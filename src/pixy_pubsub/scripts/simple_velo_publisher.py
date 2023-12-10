#!/usr/bin/env python2
import Jetson.GPIO as GPIO
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)
rospy.init_node('cmd_velo_publisher', anonymous = True)
rate = rospy.Rate(10)
en = False
#pixy.init()
#pixy.change_prog("video")

'''def publish_message():
    pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)
    rospy.init_node('cmd_velo_publisher', anonymous = True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
       # rospy.loginfo(Twist_data)
       #pub.publish(Twist_data)
       rate.sleep()'''

def en_interrupt(ENBUTTON):
    global en
    en = True
    rospy.loginfo("bp registered")

def main():
    try:
        ENBUTTON = 15
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(ENBUTTON, GPIO.IN)
        GPIO.add_event_detect(ENBUTTON, GPIO.RISING, callback = en_interrupt,bouncetime = 10)
        move_cmd = Twist()
        global en
        en = False
        rospy.loginfo("primed")
        while en == False:
            move_cmd.linear.x = 0
            move_cmd.angular.z = 0
            pub.publish(move_cmd)
            rate.sleep()
            rospy.loginfo(GPIO.input(ENBUTTON))
        now = rospy.Time.now()
        move_cmd.linear.x = 0.6
        move_cmd.angular.z = 0
        while rospy.Time.now() < now + rospy.Duration.from_sec(5):
            pub.publish(move_cmd)
            rate.sleep()
        move_cmd.linear.x = 0
        move_cmd.angular.z = 0.1
        now = rospy.Time.now()
        while rospy.Time.now() < now + rospy.Duration.from_sec(5):
            pub.publish(move_cmd)
            rate.sleep()
       # publish_message()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("shutting down ...")
        rospy.signal_shutdown("KeyboardInterrupt")
    finally:
        GPIO.cleanup()
        rospy.spin()

if __name__ == '__main__':
    main()


#!/usr/bin/env python
import roslib; roslib.load_manifest('hector_keyboard_teleop')
import rospy
from geometry_msgs.msg import Twist
import sys

twist = Twist()

flag = 1

def direction():
    value = raw_input('w,s,a,d,z,c,.: ')
    switcher = {'a':(1,0,0,0),'d':(-1,0,0,0),'w':(0,1,0,0),'s':(0,-1,0,0),'.':(0,0,0,0),'z':(0,0,1,0),'c':(0,0,-1,0)}
    val = switcher[value]
    twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z = val
    #print twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z
    flag = 0
    return twist

def keyboard():
    pub = rospy.Publisher('cmd_vel',Twist, queue_size=1)
    rospy.init_node('hector_keyboard',anonymous=True)
    rate = rospy.Rate(1) 
    while not rospy.is_shutdown():
        twist = direction()
        flag = 1
        twist = direction()
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        keyboard()
    except rospy.ROSInterruptException:
        pass

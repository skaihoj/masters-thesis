#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

rospy.init_node('teleop')
pub = rospy.Publisher('key_vel', Twist, queue_size = 1)
msg_out = Twist()


while not rospy.is_shutdown():
    msg_in = rospy.wait_for_message('joy', Joy)
    msg_out.linear.x = msg_in.axes[1] - (msg_in.axes[5] -1)*10
    msg_out.angular.z = -msg_in.axes[0]
    pub.publish(msg_out)
     


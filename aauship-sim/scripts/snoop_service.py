#!/usr/bin/env python

from gazebo_msgs.srv import SpawnModel
import rospy

def handle_add_two_ints(req):
    print req
    return SpawnModel._response_class(True, 'test')

def add_two_ints_server():
    rospy.init_node('service_snooper')
    s = rospy.Service('gazebo/spawn_sdf_model', SpawnModel, handle_add_two_ints)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()


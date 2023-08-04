#!/usr/bin/env python3

from chromebot_controller.simple_controller import SimpleController
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import numpy as np


if __name__ == '__main__':
    rospy.init_node("simple_controller_node")
    # Access the wheel radius and wheel seperation from the parameter server
    wheel_radius = rospy.get_param('~wheel_radius') # ~ represents the private namespace
    wheel_separation = rospy.get_param('~wheel_separation') # ~ represents the private namespace
    simple_controller = SimpleController(wheel_separation, wheel_radius)
    rospy.spin()
#! /usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler


rpy = quaternion_from_euler(0.043691,0.023889,-0.000144)

print rpy
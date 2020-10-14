#!/usr/bin/env python
#e-yantra task0 by @dhanuzch
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
PI = 3.1415926535897
ANGLE = 0.0000
#To get pose and then calc angle
def pose_callback(pose):
    global ANGLE
    theta = pose.theta
    if theta > 0:
        ANGLE = theta * (180/PI)
    if theta < 0:
        ANGLE = (theta * (180/PI)) + 360  
#to move the turtle
def circle():
    rospy.init_node('eyantra_task0', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    speed = 1.5
    radius = 2
    angular_velocity = speed/radius
    vel_msg = Twist()

    while not rospy.is_shutdown():
        vel_msg.linear.x = speed
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = angular_velocity
        pub.publish(vel_msg)

        #To stop the robot  
        while ANGLE >= 359.2:
            print "Reached @", ANGLE
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            pub.publish(vel_msg)
            rospy.sleep(1)
            rospy.spin()
            break

if __name__ == '__main__':
    try:
        circle()
    except rospy.ROSInterruptException:
        pass
    
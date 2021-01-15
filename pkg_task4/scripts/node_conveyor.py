#! /usr/bin/env python

import rospy
import sys
import copy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

from hrwros_gazebo.msg import LogicalCameraImage
from pkg_task4.msg import conveyor

from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import conveyorBeltPowerMsgRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsgResponse
#TODO: change msg to srv
def conveyor_control(conveyor_speed):
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        try:
            request = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
            conveyor_req = conveyorBeltPowerMsgRequest(conveyor_speed)
            return request(conveyor_req)
        except rospy.ServiceException:
            rospy.logerr ("Failed to start conveyor")


def logical_camera_clbk(msg):
    conveyorpub = rospy.Publisher('eyrc/conveyor_msg', conveyor, queue_size=1)
    conv_msg = conveyor()

    #get output from logical_camera_2 {rostopic echo /eyrc/vb/logical_camera_2}
    modelmsg = msg.models
    #print modelmsg
    if len(modelmsg) == 0:
        conveyor_control(70)
        #rospy.loginfo("False")
        conv_msg.conv_status = False
        conveyorpub.publish(conv_msg)


    if len(modelmsg) != 0:
        modelpose = msg.models[0].pose.position.y
        pkg_type = msg.models[0]
        print (pkg_type)

        if modelpose <= 0:
            conveyor_control(0)
            if pkg_type != "ur5":
                conv_msg.conv_status = True
                conv_msg.pkg_id = pkg_type
                conveyorpub.publish(conv_msg)
    
def main():
    rospy.init_node('conveyor_control', anonymous=True)
    while not rospy.is_shutdown():
        rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, logical_camera_clbk)
        rospy.spin()    

if __name__ == '__main__':
    main()

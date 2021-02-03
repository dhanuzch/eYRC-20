#! /usr/bin/env python

import rospy
import sys
import copy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

from std_msgs.msg import Bool, String
from pkg_vb_sim.msg import LogicalCameraImage

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
    statuspub = rospy.Publisher('eyrc/armsignal', Bool, queue_size=1)
    modelpub = rospy.Publisher('eyrc/pkgid', String, queue_size=1)


    #get output from logical_camera_2 {rostopic echo /eyrc/vb/logical_camera_2}
    modelmsg = msg.models
    if len(modelmsg) == 0:
        conveyor_control(70)
        rospy.loginfo("False")
        statuspub.publish(False)


    if len(modelmsg) != 0:
        modelpose = msg.models[0].pose.position.y
        pkg_id = msg.models[0].type
        print pkg_id
        if modelpose <= 0:
            conveyor_control(0)
            if pkg_id != "ur5":
                statuspub.publish(True)
                modelpub.publish(pkg_id)
    
def main():
    rospy.init_node('conveyor_control', anonymous=True)
    while not rospy.is_shutdown():
        rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, logical_camera_clbk)
        rospy.spin()    

if __name__ == '__main__':
    main()

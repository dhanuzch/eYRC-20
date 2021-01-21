#! /usr/bin/env python

import rospy
import sys
import copy

from hrwros_gazebo.msg import LogicalCameraImage
from pkg_task4.msg import conveyor

from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import conveyorBeltPowerMsgRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsgResponse

class conveyor_class():
    def __init__(self):
        self.conveyorpub = rospy.Publisher('eyrc/conveyor_msg', conveyor, queue_size=1)

    def conveyor_control(self, conveyor_speed):
            rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
            try:
                request = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
                conveyor_req = conveyorBeltPowerMsgRequest(conveyor_speed)
                return request(conveyor_req)
            except rospy.ServiceException:
                rospy.logerr ("Failed to start conveyor")


    def logical_camera_clbk(self, msg):
        #get output from logical_camera_2 {rostopic echo /eyrc/vb/logical_camera_2}
        conv_msg = conveyor()

        modelmsg = msg.models
        pkg_existence_check = len(modelmsg)
        print (pkg_existence_check)
        #print (modelmsg)

        if pkg_existence_check == 0:
            #print "no package under logical camera"
            self.conveyor_control(55)
            conv_msg.conv_status = True
            self.conveyorpub.publish(conv_msg)

        if pkg_existence_check != 0:
            modelpose = msg.models[0].pose.position.y
            pkg_type = msg.models[0].type
            if modelpose <= 0:
                if pkg_type != "ur5_2" and pkg_type != "ur5":                       
                    self.conveyor_control(0)
                    print (pkg_type)
                    conv_msg.conv_status = False
                    conv_msg.pkg_id = pkg_type
                    self.conveyorpub.publish(conv_msg)
def main():
    rospy.init_node('node_conveyor_control', anonymous=True)
    conveyor = conveyor_class()

    rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, conveyor.logical_camera_clbk)
    rospy.spin()

if __name__ == '__main__':
    conv_msg = conveyor()        

    main()

#! /usr/bin/env python

import rospy
import sys
import copy

import tf2_ros
import tf2_msgs.msg

from hrwros_gazebo.msg import LogicalCameraImage

from pkg_task4.msg import conveyor
from std_msgs.msg import Int16MultiArray

from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import conveyorBeltPowerMsgRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsgResponse

class conveyor_class():
    def __init__(self):
        self.conveyorpub = rospy.Publisher('eyrc/conveyor_msg', conveyor, queue_size=1)
        self.target_frame_pub = rospy.Publisher('eyrc/target_frame', Int16MultiArray, queue_size=1)
        self.rate = rospy.Rate(10)
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)
        pkg_details = rospy.get_param("/pkg_details")
        self.pkg_type = None
        pkg_dict = next((item for item in pkg_details if item["pkg_id"] == self.pkg_type), None)
        if pkg_dict == None:
            pass
        else:
            self.target_frame = (self.pkg_dict["target_frame"])

    def func_tf(self, arg_frame_1, arg_frame_2):
        try:
            trans = self._tfBuffer.lookup_transform(arg_frame_1, arg_frame_2, rospy.Time(0))
            tf_dict = {
            "px" : trans.transform.translation.x,
            "py" : trans.transform.translation.y,
            "pz" : trans.transform.translation.z,
            "ox" : trans.transform.rotation.x,
            "oy" : trans.transform.rotation.y,
            "oz" : trans.transform.rotation.z,
            "ow" : trans.transform.rotation.w
            }
            return tf_dict

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("TF error")

    def target_frame_call(self):
        reference_frame = "world"
        pkg_world_tf = self.func_tf(reference_frame, self.target_frame)
        #print "data type of pkg_world_tf:", type(pkg_world_tf), len(pkg_world_tf)
        x = (pkg_world_tf["px"])
        y = (pkg_world_tf["py"])
        z = (pkg_world_tf["pz"]) + 0.20
        xyz_list = [x, y, z]
        print xyz_list
        self.target_frame_pub.publish(xyz_list)

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
            self.conveyor_control(90)
            conv_msg.conv_status = True
            self.conveyorpub.publish(conv_msg)
            self.rate.sleep()

        if pkg_existence_check != 0:
            modelpose = msg.models[0].pose.position.y
            pkg_type = msg.models[0].type
            self.pkg_type = pkg_type
            if modelpose <= 0:
                self.conveyor_control(0)
                self.target_frame_call()

                if pkg_type != "ur5_2" and pkg_type != "ur5":
                    print (pkg_type)
                    conv_msg.conv_status = False
                    conv_msg.pkg_id = pkg_type
                    self.conveyorpub.publish(conv_msg)
                    self.rate.sleep()

def main():
    rospy.init_node('node_conveyor_control', anonymous=True)
    conveyor = conveyor_class()

    rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, conveyor.logical_camera_clbk)
    rospy.spin()

if __name__ == '__main__':
    conv_msg = conveyor()        

    main()

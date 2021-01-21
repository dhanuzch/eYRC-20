#! /usr/bin/env python

import rospy
import sys
import copy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math

import tf2_ros
import tf2_msgs.msg

from pkg_task4.msg import conveyor

from pkg_vb_sim.msg import LogicalCameraImage

from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import vacuumGripperRequest

target_frame = None
class CartesianPath:

    # Constructor
    def __init__(self):
        self._robot_ns = '/ur5_2'
        self._planning_group = "manipulator"
        
        moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._group.set_planning_time(10)

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        # Attribute to store computed trajectory by the planner	
        self._computed_plan = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def go_to_pose(self, arg_pose):

        self._group.get_current_pose().pose

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose

        self._group.get_current_joint_values()

        return flag_plan

class manipulation:
    def __init__(self):
        param_config_vacuum_gripper = rospy.get_param('config_vacuum_gripper_ur5_2')
        
        self._vacuum_gripper_model_name = param_config_vacuum_gripper['vacuum_gripper_model_name']
        self._vacuum_gripper_link_name = param_config_vacuum_gripper['vacuum_gripper_link_name']
        
        self._object_link_name = param_config_vacuum_gripper['attachable_object_link_name']
        
        self._attachable_object_prefix = param_config_vacuum_gripper['attachable_object_prefix']
        self._attachable_object_delimiter = param_config_vacuum_gripper['attachable_object_delimiter']
        self._logical_camera_topic_name = param_config_vacuum_gripper['logical_camera_topic_name']

        self.pkg_color = None

        self._object_model_name = ""

        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)
        #self.target_frame = None
        #self.loop_check = None
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

    def attach_box(self):
        box_model_name = self._object_model_name

        print "box model name attaching:", (box_model_name)
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
        try:
            request = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
            print ("Activating gripper")
            eef_req = vacuumGripperRequest(True)
            return request(eef_req)
        except rospy.ServiceException:
            rospy.logerr ("Failed to activate gripper")

    def detach_box(self):
        box_model_name = self._object_model_name

        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
        try:
            request = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
            print ("Deactivating gripper")
            eef_req = vacuumGripperRequest(False)
            return request(eef_req)
        except:
            rospy.logerr ("Failed to deactivate gripper")

    def ur5_camera_clbk(self, pkg_type):
        number_models = len(pkg_type.models)

        for i in range(0, number_models):
            name_model = pkg_type.models[i].type
            
            lst_name_model = name_model.split(self._attachable_object_delimiter)
            
            if(lst_name_model[0] == self._attachable_object_prefix):
                #rospy.loginfo( '\033[94m' + " Package name: {}".format(name_model) + '\033[0m')
                self._object_model_name = name_model
                break

    def conveyor_msg(self, msg):
        pkg_id_for_tf = msg.pkg_id
        conv_status = msg.conv_status
        pkg_details = rospy.get_param("/pkg_details")
        pkg_dict = next((item for item in pkg_details if item["pkg_id"] == pkg_id_for_tf), None)
        if pkg_dict == None:
            pass
        else:
            self.pkg_color = (pkg_dict["color"])


        if conv_status == True:
            print "No package detected"

        if conv_status == False:
            self.arm_move_to_box(pkg_id_for_tf)
            #self.loop_check = True


    def arm_move_to_box(self, pkg_id_for_tf):
        
        reference_frame = "world"
        camera_name = "logical_camera_2_"
        suffix = "_frame"

        target_frame = camera_name+pkg_id_for_tf+suffix
        print "*****************************************"
        print "pkg color:", self.pkg_color
        print "target_frame:", target_frame
        print "*****************************************"
    
        pkg_world_tf = self.func_tf(reference_frame, target_frame)

        #while self.loop_check == True:
        if target_frame != "logical_camera_2__frame" and target_frame != "logical_camera_2_ur5_frame" and target_frame != None:
            #print "data type of pkg_world_tf:", type(pkg_world_tf), len(pkg_world_tf)
            x = (pkg_world_tf["px"])
            y = (pkg_world_tf["py"])
            z = (pkg_world_tf["pz"]) + 0.20

            """
            x = -0.8
            y = -0.04
            z = 1.20
            """
            pkg_pose = geometry_msgs.msg.Pose()
            pkg_pose.position.x = x
            pkg_pose.position.y = y
            pkg_pose.position.z = z
            # This to keep EE parallel to Ground Plane
            pkg_pose.orientation.x = -0.5
            pkg_pose.orientation.y = -0.5
            pkg_pose.orientation.z = 0.5
            pkg_pose.orientation.w = 0.5
            
            try:
                arm_to_box_Status = ur5.go_to_pose(pkg_pose)
                print "arm_to_box_Status:", arm_to_box_Status
                if arm_to_box_Status == True:
                    rospy.sleep(1)

                    if len(self._object_model_name) >=1:
                        if self._object_model_name == pkg_id_for_tf:
                            self.attach_box()
                            rospy.sleep(1)
                            arm_to_bin_Status = self.arm_move_func(self.pkg_color)
                            print "arm_to_bin_Status:", arm_to_bin_Status
                            self.detach_box()

            except:
                print "Failed to pick and drop"
                        
    def arm_move_func(self, pkg_color):
        if pkg_color == "red":
            self.joint_angle_set(80) #red bin joint angle shldr_pan

        if pkg_color == "yellow":
            self.joint_angle_set(0) #yellow bin joint angle shldr_pan

        if pkg_color == "green":
            self.joint_angle_set(-90) #green bin joint angle shldr_pan

    def joint_angle_set(self, joint1):
        lst_joint_angles = [math.radians(joint1),
                          math.radians(-45),
                          math.radians(75),
                          math.radians(-120),
                          math.radians(-90),
                          math.radians(0)]

        ur5.set_joint_angles(lst_joint_angles)
    
def main():
    global target_frame
    rospy.Subscriber("/eyrc/conveyor_msg", conveyor, Manipulation.conveyor_msg)
    rospy.Subscriber("eyrc/vb/ur5_2/vacuum_gripper/logical_camera/ur5_2", LogicalCameraImage, Manipulation.ur5_camera_clbk, queue_size=1)
    rospy.spin()    

if __name__ == '__main__':
    rospy.init_node('node_ur5_2', anonymous=True)
    Manipulation = manipulation()
    ur5 = CartesianPath()
    main()


#! /usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg

import yaml
import os
import math
import time
import sys
import copy

from std_srvs.srv import Empty

from std_msgs.msg import String

from moveit_commander.conversions import pose_to_list
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import vacuumGripperRequest

class Ur5Moveit:

    # Constructor
    def __init__(self):
        self._robot_ns = '/ur5_1'
        self._planning_group = "manipulator"
        
        moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''


        # Attribute to store computed trajectory by the planner	
        self._computed_plan = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')


        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task4')
        self._file_path = self._pkg_path + '/config/saved_trajectories_final/'
        rospy.loginfo( "Package Path: {}".format(self._file_path) )


        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def clear_octomap(self):
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        if (flag_plan == True):
            pass
            # rospy.loginfo(
            #     '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            pass
            # rospy.logerr(
            #     '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

        number_attempts = 0
        flag_success = False
        
        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # self.clear_octomap()


    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        file_path = arg_file_path + arg_file_name
        
        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)
        
        ret = self._group.execute(loaded_plan)
        # rospy.logerr(ret)
        return ret

    
    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        number_attempts = 0
        flag_success = False

        while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # # self.clear_octomap()
        
        return True
    def wait_for_state_update(self, box_is_known=False, box_is_attached=False):
            box_name = self._box_name
            scene = self._scene

            start = rospy.get_time()
            seconds = rospy.get_time()
            while (seconds - start < 0.1) and not rospy.is_shutdown():
                attached_objects = scene.get_attached_objects([box_name])
                is_attached = len(attached_objects.keys()) > 0

                is_known = box_name in scene.get_known_object_names()

                # Test if we are in the expected state
                if (box_is_attached == is_attached) and (box_is_known == is_known):
                    return True

                # Sleep so that we give other threads time on the processor
                rospy.sleep(0.1)
                seconds = rospy.get_time()

            # If we exited the while loop without returning then we timed out
            return False

    def add_box(self):
        scene = self._scene

        box_pose00 = geometry_msgs.msg.PoseStamped()
        box_pose00.header.frame_id = self._planning_frame
        box_pose00.pose.position.x = 0.28
        box_pose00.pose.position.y = -0.41
        box_pose00.pose.position.z = 1.92 
        box_pose01 = geometry_msgs.msg.PoseStamped()
        box_pose01.header.frame_id = self._planning_frame
        box_pose01.pose.position.x = 0
        box_pose01.pose.position.y = -0.41
        box_pose01.pose.position.z = 1.92  
        box_pose02 = geometry_msgs.msg.PoseStamped()
        box_pose02.header.frame_id = self._planning_frame
        box_pose02.pose.position.x = -0.28
        box_pose02.pose.position.y = -0.41
        box_pose02.pose.position.z = 1.92  
        box_pose10 = geometry_msgs.msg.PoseStamped()
        box_pose10.header.frame_id = self._planning_frame
        box_pose10.pose.position.x = 0.28
        box_pose10.pose.position.y = -0.41
        box_pose10.pose.position.z = 1.65
        box_pose11 = geometry_msgs.msg.PoseStamped()
        box_pose11.header.frame_id = self._planning_frame
        box_pose11.pose.position.x = 0.0
        box_pose11.pose.position.y = -0.41
        box_pose11.pose.position.z = 1.65  
        box_pose12 = geometry_msgs.msg.PoseStamped()
        box_pose12.header.frame_id = self._planning_frame
        box_pose12.pose.position.x = -0.28
        box_pose12.pose.position.y = -0.41
        box_pose12.pose.position.z = 1.65 
        box_pose20 = geometry_msgs.msg.PoseStamped()
        box_pose20.header.frame_id = self._planning_frame
        box_pose20.pose.position.x = 0.28
        box_pose20.pose.position.y = -0.41
        box_pose20.pose.position.z = 1.43  
        box_pose21 = geometry_msgs.msg.PoseStamped()
        box_pose21.header.frame_id = self._planning_frame
        box_pose21.pose.position.x = 0.0
        box_pose21.pose.position.y = -0.41
        box_pose21.pose.position.z = 1.43
        box_pose22 = geometry_msgs.msg.PoseStamped()
        box_pose22.header.frame_id = self._planning_frame
        box_pose22.pose.position.x = -0.28
        box_pose22.pose.position.y = -0.41
        box_pose22.pose.position.z = 1.43
        box_pose30 = geometry_msgs.msg.PoseStamped()
        box_pose30.header.frame_id = self._planning_frame
        box_pose30.pose.position.x = 0.28
        box_pose30.pose.position.y = -0.41
        box_pose30.pose.position.z = 1.197 
        box_pose31 = geometry_msgs.msg.PoseStamped()
        box_pose31.header.frame_id = self._planning_frame
        box_pose31.pose.position.x = 0.0
        box_pose31.pose.position.y = -0.41
        box_pose31.pose.position.z = 1.197
        box_pose32 = geometry_msgs.msg.PoseStamped()
        box_pose32.header.frame_id = self._planning_frame
        box_pose32.pose.position.x = -0.28
        box_pose32.pose.position.y = -0.41
        box_pose32.pose.position.z = 1.197
        
     
        scene.add_box("packagen00", box_pose00, size=(0.15, 0.15, 0.15))
        scene.add_box("packagen01", box_pose01, size=(0.15, 0.15, 0.15))
        scene.add_box("packagen02", box_pose02, size=(0.15, 0.15, 0.15))
        scene.add_box("packagen10", box_pose10, size=(0.15, 0.15, 0.15))
        scene.add_box("packagen11", box_pose11, size=(0.15, 0.15, 0.15))
        scene.add_box("packagen12", box_pose12, size=(0.15, 0.15, 0.15))
        scene.add_box("packagen20", box_pose20, size=(0.15, 0.15, 0.15))
        scene.add_box("packagen21", box_pose21, size=(0.15, 0.15, 0.15))
        scene.add_box("packagen22", box_pose22, size=(0.15, 0.15, 0.15))
        scene.add_box("packagen30", box_pose30, size=(0.15, 0.15, 0.15))
        scene.add_box("packagen31", box_pose31, size=(0.15, 0.15, 0.15))
        scene.add_box("packagen32", box_pose32, size=(0.15, 0.15, 0.15))

    def attach_box(self, box_name):
        self._scene.attach_box(self._eef_link, box_name, touch_links='vacuum_gripper_link')
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        try:
            request = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
            print ("Activating gripper")
            eef_req = vacuumGripperRequest(True)
            return request(eef_req)
        except rospy.ServiceException:
            rospy.logerr ("Failed to activate gripper")

        return self.wait_for_state_update(box_is_attached=True, box_is_known=False)

    def detach_box(self, box_name):
        self._scene.remove_attached_object(self._eef_link, name=box_name)
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        try:
            request = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
            print ("***Deactivating gripper***")
            eef_req = vacuumGripperRequest(False)
            return request(eef_req)
        except:
            return 0
            
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False)

    def remove_box(self, box_name):
        scene = self._scene
        scene.remove_world_object(box_name)
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False)

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()
    ur5.add_box()
    

    rospy.logwarn("1")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '1_ze00.yaml', 5)
    box_name00 = "packagen00"
    ur5.attach_box(box_name00)


    rospy.logwarn("2")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '2_00co.yaml', 5)
    ur5.detach_box(box_name00)
    ur5.remove_box(box_name00)

    rospy.logwarn("3")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '3_co01.yaml', 5)
    box_name01 = "packagen01"
    ur5.attach_box(box_name01)


    rospy.logwarn("4")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '4_01co.yaml', 5)
    ur5.detach_box(box_name01)
    ur5.remove_box(box_name01)


    rospy.logwarn("5")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '5_co02.yaml', 5)
    box_name02 = "packagen02"
    ur5.attach_box(box_name02)

    
    rospy.logwarn("6")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '6_02co.yaml', 5)
    ur5.detach_box(box_name02)
    ur5.remove_box(box_name02)


    rospy.logwarn("7")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '7_co10.yaml', 5)
    box_name10 = "packagen10"
    ur5.attach_box(box_name10)


    rospy.logwarn("8")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '8_10co.yaml', 5)
    ur5.detach_box(box_name10)
    ur5.remove_box(box_name10)


    rospy.logwarn("9")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '9_co11.yaml', 5)
    box_name11 = "packagen11"
    ur5.attach_box(box_name11)

    
    rospy.logwarn("10")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '10_11co.yaml', 5)
    ur5.detach_box(box_name11)
    ur5.remove_box(box_name11)


    rospy.logwarn("11")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '11_co12.yaml', 5)
    box_name12 = "packagen12"
    ur5.attach_box(box_name12)


    rospy.logwarn("12")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '12_12co.yaml', 5)
    ur5.detach_box(box_name12)
    ur5.remove_box(box_name12)

    rospy.logwarn("13")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '13_co20.yaml', 5)
    box_name20 = "packagen20"
    ur5.attach_box(box_name20)

    
    rospy.logwarn("14")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '14_20co.yaml', 5)
    ur5.detach_box(box_name20)
    ur5.remove_box(box_name20)


    rospy.logwarn("15")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '15_co21.yaml', 5)
    box_name21 = "packagen21"
    ur5.attach_box(box_name21)


    rospy.logwarn("16")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '16_21co.yaml', 5)
    ur5.detach_box(box_name21)
    ur5.remove_box(box_name21)


    rospy.logwarn("17")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '17_co22.yaml', 5)
    box_name22 = "packagen22"
    ur5.attach_box(box_name22)

    
    rospy.logwarn("18")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '18_22co.yaml', 5)
    ur5.detach_box(box_name22)
    ur5.remove_box(box_name22)


    rospy.logwarn("19")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '19_co30.yaml', 5)
    box_name30 = "packagen30"
    ur5.attach_box(box_name30)


    rospy.logwarn("20")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '20_30co.yaml', 5)
    ur5.detach_box(box_name30)
    ur5.remove_box(box_name30)


    rospy.logwarn("21")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '21_co31.yaml', 5)
    box_name31 = "packagen31"
    ur5.attach_box(box_name31)

    
    rospy.logwarn("22")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '22_31co.yaml', 5)
    ur5.detach_box(box_name31)
    ur5.remove_box(box_name31)


    rospy.logwarn("23")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '23_co32.yaml', 5)
    box_name32 = "packagen32"
    ur5.attach_box(box_name32)


    rospy.logwarn("24")
    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '24_32co.yaml', 5)
    ur5.detach_box(box_name32)
    ur5.remove_box(box_name32)

    rospy.spin()

    

if __name__ == '__main__':
    rospy.init_node('node_ur5_1', anonymous=True)
    main()


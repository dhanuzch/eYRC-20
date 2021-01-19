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

class Ur5Moveit:

    # Constructor
    def __init__(self, arg_robot_name):

        rospy.init_node('node_moveit_eg7', anonymous=True)

        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"
        
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
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

        
    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit(sys.argv[1])

    
    while not rospy.is_shutdown():

        rospy.logwarn("1")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '1_ze00.yaml', 5)
        
        rospy.logwarn("2")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '2_00co.yaml', 5)

        rospy.logwarn("3")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '3_co01.yaml', 5)

        rospy.logwarn("4")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '4_01co.yaml', 5)

        rospy.logwarn("5")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '5_co02.yaml', 5)
        
        rospy.logwarn("6")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '6_02co.yaml', 5)

        rospy.logwarn("7")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '7_co10.yaml', 5)

        rospy.logwarn("8")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '8_10co.yaml', 5)

        rospy.logwarn("9")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '9_co11.yaml', 5)
        
        rospy.logwarn("10")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '10_11co.yaml', 5)

        rospy.logwarn("11")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '11_co12.yaml', 5)

        rospy.logwarn("12")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '12_12co.yaml', 5)

        rospy.logwarn("13")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '13_co20.yaml', 5)
        
        rospy.logwarn("14")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '14_20co.yaml', 5)

        rospy.logwarn("15")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '15_co21.yaml', 5)

        rospy.logwarn("16")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '16_21co.yaml', 5)

        rospy.logwarn("11")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '17_co22.yaml', 5)
        
        rospy.logwarn("18")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '18_22co.yaml', 5)

        rospy.logwarn("19")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '19_co30.yaml', 5)

        rospy.logwarn("20")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '20_30co.yaml', 5)

        rospy.logwarn("21")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '21_co31.yaml', 5)
        
        rospy.logwarn("22")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '22_31co.yaml', 5)

        rospy.logwarn("23")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '23_co32.yaml', 5)

        rospy.logwarn("24")
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '24_32co.yaml', 5)

    del ur5



if __name__ == '__main__':
    main()


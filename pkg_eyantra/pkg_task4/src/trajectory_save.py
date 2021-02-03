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
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import vacuumGripperRequest

class Ur5Moveit:

    # Constructor
    def __init__(self, arg_robot_name):


        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"
        
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()
        #self._group.setPlannerId("RRTConnectkConfigDefault")
        #self._group.setPlanningTime(10)
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        self._group.set_planning_time(99)

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
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
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

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
            box_name = self._box_name
            scene = self._scene

            start = rospy.get_time()
            seconds = rospy.get_time()
            #while (seconds - start < 1) and not rospy.is_shutdown():
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(1)
            seconds = rospy.get_time()

            # If we exited the while loop without returning then we timed out
            return False
    """
    def attach_box(self, box_name):
        self._scene.attach_box(self._eef_link, box_name, touch_links='vacuum_gripper_link')
        #request = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
        print ("Activating gripper"), timeout=4
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False)



    def detach_box(self, box_name):
        self._scene.remove_attached_object(self._eef_link, name=box_name)
        #request = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
        print ("Deactivating gripper")
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False)
        """

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

    def remove_box(self, box_name):
        scene = self._scene
        scene.remove_world_object(box_name)
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=4)
    
    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():
    ur5 = Ur5Moveit(sys.argv[1])
    ur5.add_box()

    box_name00 = "packagen00"
    box_name01 = "packagen01"
    box_name02 = "packagen02"
    box_name10 = "packagen10"
    box_name11 = "packagen11"
    box_name12 = "packagen12"
    box_name20 = "packagen20"
    box_name21 = "packagen21"
    box_name22 = "packagen22"
    box_name30 = "packagen30"
    box_name31 = "packagen31"
    box_name32 = "packagen32"

    lst_joint_angles_co = [math.radians(0),
                          math.radians(-156),
                          math.radians(-5),
                          math.radians(-109),
                          math.radians(90),
                          math.radians(0)]


    lst_joint_angles_00 = [math.radians(-54),
                          math.radians(-74),
                          math.radians(15),
                          math.radians(-120),
                          math.radians(-127),
                          math.radians(103)]

    lst_joint_angles_01 = [math.radians(-127),
                          math.radians(-91),
                          math.radians(27),
                          math.radians(-114),
                          math.radians(-54),
                          math.radians(101)]

    lst_joint_angles_02 = [math.radians(-163),
                          math.radians(-73),
                          math.radians(14),
                          math.radians(-115),
                          math.radians(-18),
                          math.radians(97)]

    lst_joint_angles_10 = [math.radians(-53),
                          math.radians(-97),
                          math.radians(86),
                          math.radians(-167),
                          math.radians(-129),
                          math.radians(103)]

    lst_joint_angles_11 = [math.radians(122),
                          math.radians(-64),
                          math.radians(-97),
                          math.radians(-16),
                          math.radians(56),
                          math.radians(100)]

    lst_joint_angles_12 = [math.radians(56),
                          math.radians(-83),
                          math.radians(-85),
                          math.radians(-11),
                          math.radians(123),
                          math.radians(103)]

    lst_joint_angles_20 = [math.radians(-53),
                          math.radians(-97),
                          math.radians(90),
                          math.radians(6),
                          math.radians(128),
                          math.radians(-79)]

    lst_joint_angles_21 = [math.radians(122),
                          math.radians(-62),
                          math.radians(-104),
                          math.radians(166),
                          math.radians(-56),
                          math.radians(-97)]

    lst_joint_angles_22 = [math.radians(56),
                          math.radians(-85),
                          math.radians(-116),
                          math.radians(22),
                          math.radians(122),
                          math.radians(102)]

    lst_joint_angles_30 = [math.radians(-55),
                          math.radians(-95),
                          math.radians(119),
                          math.radians(-26),
                          math.radians(126),
                          math.radians(-79)]

    lst_joint_angles_31 = [math.radians(-125),
                          math.radians(-119),
                          math.radians(138),
                          math.radians(-21),
                          math.radians(57),
                          math.radians(-77)]

    lst_joint_angles_32 = [math.radians(56),
                          math.radians(-106),
                          math.radians(-132),
                          math.radians(57),
                          math.radians(123),
                          math.radians(105)]

    # 1-------------------------------------------------------------------------
    ur5.hard_set_joint_angles(lst_joint_angles_00, 5)
    ur5.attach_box(box_name00)

    file_name = '1_ze00.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(2)

    # 2-------------------------------------------------------------------------
    ur5.hard_set_joint_angles(lst_joint_angles_co, 5)
    ur5.detach_box(box_name00)
    rospy.sleep(.5)
    ur5.remove_box(box_name00)

    file_name = '2_00co.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(2)

    # 3-------------------------------------------------------------------------
    ur5.hard_set_joint_angles(lst_joint_angles_01, 5)
    ur5.attach_box(box_name01)

    file_name = '3_co01.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(2)

    # 4-------------------------------------------------------------------------
    ur5.hard_set_joint_angles(lst_joint_angles_co, 5)
    ur5.detach_box(box_name01)
    rospy.sleep(.5)
    ur5.remove_box(box_name01)

    file_name = '4_01co.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(2)

    # 5-------------------------------------------------------------------------
    ur5.hard_set_joint_angles(lst_joint_angles_02, 5)
    ur5.attach_box(box_name02)

    file_name = '5_co02.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(2)

    # 6-------------------------------------------------------------------------
    ur5.hard_set_joint_angles(lst_joint_angles_co, 5)
    ur5.detach_box(box_name02)
    rospy.sleep(.5)
    ur5.remove_box(box_name02)

    file_name = '6_02co.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(2)

    # 7-------------------------------------------------------------------------
    ur5.hard_set_joint_angles(lst_joint_angles_10, 5)
    ur5.attach_box(box_name10)

    file_name = '7_co10.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(2)

    # 8-------------------------------------------------------------------------
    ur5.hard_set_joint_angles(lst_joint_angles_co, 5)
    ur5.detach_box(box_name10)
    rospy.sleep(.5)
    ur5.remove_box(box_name10)

    file_name = '8_10co.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(2)
    # 9-------------------------------------------------------------------------
    ur5.hard_set_joint_angles(lst_joint_angles_11, 5)
    ur5.attach_box(box_name11)

    file_name = '9_co11.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(2)

    # 10-------------------------------------------------------------------------
    ur5.hard_set_joint_angles(lst_joint_angles_co, 5)
    ur5.detach_box(box_name11)
    rospy.sleep(.5)
    ur5.remove_box(box_name11)

    file_name = '10_11co.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(2)

    # 11-------------------------------------------------------------------------
    ur5.hard_set_joint_angles(lst_joint_angles_12, 5)
    ur5.attach_box(box_name12)

    file_name = '11_co12.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(2)

    # 12-------------------------------------------------------------------------
    ur5.hard_set_joint_angles(lst_joint_angles_co, 5)
    ur5.detach_box(box_name12)
    rospy.sleep(.5)
    ur5.remove_box(box_name12)

    file_name = '12_12co.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(2)
    # 13-------------------------------------------------------------------------
    ur5.hard_set_joint_angles(lst_joint_angles_20, 5)
    ur5.attach_box(box_name20)

    file_name = '13_co20.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(2)

    # 14-------------------------------------------------------------------------
    ur5.hard_set_joint_angles(lst_joint_angles_co, 5)
    ur5.detach_box(box_name20)
    rospy.sleep(.5)
    ur5.remove_box(box_name20)

    file_name = '14_20co.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(2)

    # 15-------------------------------------------------------------------------
    ur5.hard_set_joint_angles(lst_joint_angles_21, 5)
    ur5.attach_box(box_name21)

    file_name = '15_co21.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(2)

    # 16-------------------------------------------------------------------------
    ur5.hard_set_joint_angles(lst_joint_angles_co, 5)
    ur5.detach_box(box_name21)
    rospy.sleep(.5)
    ur5.remove_box(box_name21)

    file_name = '16_21co.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(2)
    # 17-------------------------------------------------------------------------
    ur5.hard_set_joint_angles(lst_joint_angles_22, 5)
    ur5.attach_box(box_name22)

    file_name = '17_co22.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(2)

    # 18-------------------------------------------------------------------------
    ur5.hard_set_joint_angles(lst_joint_angles_co, 5)
    ur5.detach_box(box_name22)
    rospy.sleep(.5)
    ur5.remove_box(box_name22)

    file_name = '18_22co.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(2)

    # 19-------------------------------------------------------------------------
    ur5.hard_set_joint_angles(lst_joint_angles_30, 5)
    ur5.attach_box(box_name30)

    file_name = '19_co30.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(2)

    # 20-------------------------------------------------------------------------
    ur5.hard_set_joint_angles(lst_joint_angles_co, 5)
    ur5.detach_box(box_name30)
    rospy.sleep(.5)
    ur5.remove_box(box_name30)

    file_name = '20_30co.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(2)

    # 21-------------------------------------------------------------------------
    ur5.hard_set_joint_angles(lst_joint_angles_31, 5)
    ur5.attach_box(box_name31)

    file_name = '21_co31.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(2)

    # 22-------------------------------------------------------------------------
    ur5.hard_set_joint_angles(lst_joint_angles_co, 5)
    ur5.detach_box(box_name31)
    rospy.sleep(.5)
    ur5.remove_box(box_name31)

    file_name = '22_31co.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(2)

    # 23-------------------------------------------------------------------------
    ur5.hard_set_joint_angles(lst_joint_angles_32, 5)
    ur5.attach_box(box_name32)

    file_name = '23_co32.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(2)

    # 24-------------------------------------------------------------------------
    ur5.hard_set_joint_angles(lst_joint_angles_co, 5)
    ur5.detach_box(box_name32)
    rospy.sleep(.5)
    ur5.remove_box(box_name32)

    file_name = '24_32co.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    rospy.sleep(2)
    
    del ur5

if __name__ == '__main__':
    rospy.init_node('node_moveit_eg6', anonymous=True)
    main()


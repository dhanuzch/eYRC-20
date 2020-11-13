#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_t2_ur5_1_pick_place', anonymous=True)

        self._planning_group = "ur5_1_planning_group"
        moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()
        self._planning_frame = self._group.get_planning_frame()
        #self._eef_link = "vacuum_gripper_link"
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self.move_group = moveit_commander.MoveGroupCommander(self._group_names)
        box_name = "package$1"


        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')


    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
            box_name = self._box_name
            scene = self._scene

            start = rospy.get_time()
            seconds = rospy.get_time()
            while (seconds - start < timeout) and not rospy.is_shutdown():
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

    def add_box(self, timeout=4):
        #0.03, 0.31, 1.97,  # position
        #0.0, 0.0, 0.0,  # rotation
        #0.15, 0.15, 0.15)  # size
        #(0.03,0.23,1.89) in piazza
        #(0.35,29,1.96) in gazebo
        #(0.04,0.47, 1.85) mine

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self._planning_frame
        box_pose.pose.position.x = 0.04
        box_pose.pose.position.y = 0.47
        box_pose.pose.position.z = 1.85   
        box_name = "box"
     
        self._scene.add_box(box_name, box_pose, size=(0.15, 0.15, 0.15))
        self._box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = 'hand'
        touch_links = self._robot.get_link_names(group=self._group_names)

        self._scene.attach_box(self._eef_link, self._box_name, self.touch_links)
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):
        self._scene.remove_attached_object(self._eef_link, name=self._box_name)
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)



if __name__ == '__main__':
    ur5 = Ur5Moveit()
    ur5.add_box()
    #Go to predefined pose tobox
    ur5.go_to_predefined_pose("toBox")
    rospy.sleep(10)
    #ur5.attach_box()    
    #Go to the predefined pose tobin
    ur5.go_to_predefined_pose("toBin")
    rospy.sleep(3)
    ur5.detach_box()    

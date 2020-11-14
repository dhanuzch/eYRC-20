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
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import vacuumGripperRequest

class Ur5Moveit:

    # Constructor
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('node_t2_ur5_1_pick_place', anonymous=True)


        self._planning_group = "ur5_1_planning_group"
        self._robot = moveit_commander.RobotCommander()
        self._group_names = self._robot.get_group_names()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group) #group_name  
        #self.move_group = moveit_commander.MoveGroupCommander(self._group_names)     
        print"___________________________________________________"
        print("group_names:", self._group_names)
        print("group:", self._group)
        #print("move_group:", self.move_group)
        print"___________________________________________________"
        self._scene = moveit_commander.PlanningSceneInterface()
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._planning_frame = self._group.get_planning_frame()
        #self._eef_link = "vacuum_gripper_link"
        self._eef_link = self._group.get_end_effector_link()


        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()
        
        self._box_name = "package$1"


        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo('\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')
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

    #def execute_plan(self, plan):
    #    move_group = self.move_group
    #    move_group.execute(plan, wait=True)

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
        box_name = self._box_name
        scene = self._scene

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
     
        scene.add_box(box_name, box_pose, size=(0.15, 0.15, 0.15))
        self._box_name=box_name #try removing this TODOTODOTODOTODOTODOTODOTODOTODOTODO
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        #'ur5_1' using link: 'vacuum_gripper_link' with model: 'package$1' using link: 'link'

        #grasping_group = self._group
        #touch_links = self._robot.get_link_names(group=grasping_group)
        #touch_links = 'vacuum_gripper_link'

        #self._scene.attach_box(self._eef_link, self._box_name, touch_links=['vacuum_gripper_link', 'link'])
        self._scene.attach_box(self._eef_link, self._box_name)
        #******************************
        #GAZEBO
        rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
        try:
            rospy.sleep(5)
            request = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
            print "***Activating gripper***"
            eef_req = vacuumGripperRequest(True)
            return request(eef_req)
        except rospy.ServiceException:
            print "***Failed to activate gripper***"
        #******************************

        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):
        self._scene.remove_attached_object(self._eef_link, name=self._box_name)
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)
        
        # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

def main():
    try:
        ur5 = Ur5Moveit()
        #typesrv="pkg_vb_sim/vacuumGripper"
        ur5.add_box()
        #Go to predefined pose tobox
        ur5.go_to_predefined_pose("toBox")
        ur5.attach_box()
        rospy.sleep(1)   
        #Go to the predefined pose tobin
        ur5.go_to_predefined_pose("toBin")
        rospy.sleep(3)
        ur5.detach_box()
    except rospy.ROSInterruptException:
        return "Failed lol"

if __name__ == '__main__':
    main()

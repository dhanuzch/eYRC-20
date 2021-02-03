#! /usr/bin/env python
v1
import rospy
import sys
import copy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

import tf2_ros
import tf2_msgs.msg

from pkg_vb_sim.msg import LogicalCameraImage

from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import conveyorBeltPowerMsgRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsgResponse

from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import vacuumGripperRequest

class CartesianPath:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg5_waypoints', anonymous=True)

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
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')


    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        # 1. Create a empty list to hold waypoints
        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)  
        wpose.position.y = waypoints[0].position.y + (trans_y)  
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5


        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))


        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")

        # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
        # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)

    
    def go_to_pose(self, arg_pose):

        self._group.get_current_pose().pose

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose

        self._group.get_current_joint_values()

        return flag_plan

class tfEcho:

    def __init__(self):
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

    def func_tf_print(self, arg_frame_1, arg_frame_2):
        try:
            trans = self._tfBuffer.lookup_transform(arg_frame_1, arg_frame_2, rospy.Time())

            rospy.loginfo(  "\n" +
                            "Translation: \n" +
                            "x: {} \n".format(trans.transform.translation.x) +
                            "y: {} \n".format(trans.transform.translation.y) +
                            "z: {} \n".format(trans.transform.translation.z) +
                            "\n" +
                            "Orientation: \n" +
                            "x: {} \n".format(trans.transform.rotation.x) +
                            "y: {} \n".format(trans.transform.rotation.y) +
                            "z: {} \n".format(trans.transform.rotation.z) +
                            "w: {} \n".format(trans.transform.rotation.w) )

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("TF error")

    def func_tf(self, arg_frame_1, arg_frame_2):
        try:
            trans = self._tfBuffer.lookup_transform(arg_frame_1, arg_frame_2, rospy.Time())
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


class manipulation:
    def __init__(self):
        self.request = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)

        param_config_vacuum_gripper = rospy.get_param('config_vacuum_gripper')
        
        self._vacuum_gripper_model_name = param_config_vacuum_gripper['vacuum_gripper_model_name']
        self._vacuum_gripper_link_name = param_config_vacuum_gripper['vacuum_gripper_link_name']
        
        self._object_model_name = ""
        self._object_link_name = param_config_vacuum_gripper['attachable_object_link_name']
        
        self._attachable_object_prefix = param_config_vacuum_gripper['attachable_object_prefix']
        self._attachable_object_delimiter = param_config_vacuum_gripper['attachable_object_delimiter']
        self._logical_camera_topic_name = param_config_vacuum_gripper['logical_camera_topic_name']

    def conveyor_control(self, conveyor_speed):
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        try:
            request = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
            conveyor_req = conveyorBeltPowerMsgRequest(conveyor_speed)
            return request(conveyor_req)
        except rospy.ServiceException:
            rospy.logerr ("Failed to start conveyor")

    def logical_camera_clbk(self, pkg_status):
        #get output from logical_camera_2 {rostopic echo /eyrc/vb/logical_camera_2}
        modelmsg = pkg_status.models
        if len(modelmsg) == 0:
            conveyor_speed = 100
            self.conveyor_control(conveyor_speed)
            print "No packages in range"

        if len(modelmsg) != 0:
            rospy.sleep(.2)
            #  Fix the small .2 seconds delay...or add (stop the package when y>=0)
            conveyor_speed = 0
            self.conveyor_control(conveyor_speed)
            self.ee_move()
    

    def ur5_camera_clbk(self, pkg_type):
        number_models = len(pkg_type.models)

        for i in range(0, number_models):
            name_model = pkg_type.models[i].type
            
            lst_name_model = name_model.split(self._attachable_object_delimiter)
            
            if(lst_name_model[0] == self._attachable_object_prefix):
                #rospy.loginfo( '\033[94m' + " Package name: {}".format(name_model) + '\033[0m')
                self._object_model_name = name_model
                break

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=0.1):
            box_model_name = self._object_model_name
            scene = ur5._scene

            start = rospy.get_time()
            seconds = rospy.get_time()
            while (seconds - start < timeout) and not rospy.is_shutdown():
                attached_objects = scene.get_attached_objects([box_model_name])
                is_attached = len(attached_objects.keys()) > 0

                is_known = box_model_name in scene.get_known_object_names()

                # Test if we are in the expected state
                if (box_is_attached == is_attached) and (box_is_known == is_known):
                    return True

                # Sleep so that we give other threads time on the processor
                rospy.sleep(0.1)
                seconds = rospy.get_time()

            # If we exited the while loop without returning then we timed out
            return False

    def ee_move(self):
        my_tf = tfEcho()

        reference_frame1 = "logical_camera_2_frame"
        target_frame1 = "logical_camera_2_packagen1_frame"
        reference_frame2 = "world"
        target_frame2 = "ur5_ee_link"

        #fix the starting delay of the arm
        rospy.sleep(.2)

        #==================
        # Pose
        #==================
        pkg_world_tf = my_tf.func_tf(reference_frame2, target_frame1)
        x = (pkg_world_tf["px"])
        y = (pkg_world_tf["py"])
        z = (pkg_world_tf["pz"]) + 0.20
        
        pkg_pose = geometry_msgs.msg.Pose()
        pkg_pose.position.x = x
        pkg_pose.position.y = y
        pkg_pose.position.z = z
        # This to keep EE parallel to Ground Plane
        pkg_pose.orientation.x = -0.5
        pkg_pose.orientation.y = -0.5
        pkg_pose.orientation.z = 0.5
        pkg_pose.orientation.w = 0.5

        attach = "attach"
        detach = "detach"
        while not rospy.is_shutdown():
            ur5.go_to_pose(pkg_pose)
            self.ee_grip(attach)
            break

        """
        print "_____________________"
        print ur5_ee_tf
        print box_model_name
        print "_____________________"
        """

    def ee_grip(self, gripperstate):
        box_model_name = self._object_model_name

        rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')

        if (gripperstate == "attach"):
            ur5._scene.attach_box(ur5._eef_link, box_model_name, touch_links='vacuum_gripper_link')
            try:
                rospy.sleep(2)
                request = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
                rospy.loginfo( '\033[94m' + " Activating gripper " '\033[0m')
                eef_req = vacuumGripperRequest(True)
                return request(eef_req)
            except rospy.ServiceException:
                rospy.logerr ("Failed to activate gripper")

            return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=0.1)

        if (gripperstate == "detach"):
            ur5._scene.remove_attached_object(ur5._eef_link, name=box_model_name)
            rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
            try:
                request = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
                rospy.loginfo( '\033[94m' + " Dectivating gripper " '\033[0m')
                eef_req = vacuumGripperRequest(False)
                return request(eef_req)
            except rospy.ServiceException:
                rospy.logerr ("Failed to deactivate gripper")


            return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=0.1)


    def to_bin(self, bin_color):
        print "lol"

def main():
    Manipulation = manipulation()
    while not rospy.is_shutdown():
        rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, Manipulation.logical_camera_clbk)
        rospy.Subscriber("eyrc/vb/ur5_1/vacuum_gripper/logical_camera/ur5_1", LogicalCameraImage, Manipulation.ur5_camera_clbk)
        rospy.spin()    

if __name__ == '__main__':
    ur5 = CartesianPath()
    main()

v3

#! /usr/bin/env python

import rospy
import sys
import copy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

import tf2_ros
import tf2_msgs.msg

from pkg_vb_sim.msg import LogicalCameraImage

from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import conveyorBeltPowerMsgRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsgResponse

from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import vacuumGripperRequest

class CartesianPath:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg5_waypoints', anonymous=True)

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
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')


    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        # 1. Create a empty list to hold waypoints
        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)  
        wpose.position.y = waypoints[0].position.y + (trans_y)  
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5


        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))


        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")

        # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
        # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)

    
    def go_to_pose(self, arg_pose):

        self._group.get_current_pose().pose

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose

        self._group.get_current_joint_values()

        return flag_plan

class tfEcho:

    def __init__(self):
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

    def func_tf_print(self, arg_frame_1, arg_frame_2):
        try:
            trans = self._tfBuffer.lookup_transform(arg_frame_1, arg_frame_2, rospy.Time())

            rospy.loginfo(  "\n" +
                            "Translation: \n" +
                            "x: {} \n".format(trans.transform.translation.x) +
                            "y: {} \n".format(trans.transform.translation.y) +
                            "z: {} \n".format(trans.transform.translation.z) +
                            "\n" +
                            "Orientation: \n" +
                            "x: {} \n".format(trans.transform.rotation.x) +
                            "y: {} \n".format(trans.transform.rotation.y) +
                            "z: {} \n".format(trans.transform.rotation.z) +
                            "w: {} \n".format(trans.transform.rotation.w) )

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("TF error")

    def func_tf(self, arg_frame_1, arg_frame_2):
        try:
            trans = self._tfBuffer.lookup_transform(arg_frame_1, arg_frame_2, rospy.Time())
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


class manipulation:
    def __init__(self):
        self.request = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)

        param_config_vacuum_gripper = rospy.get_param('config_vacuum_gripper')
        
        self._vacuum_gripper_model_name = param_config_vacuum_gripper['vacuum_gripper_model_name']
        self._vacuum_gripper_link_name = param_config_vacuum_gripper['vacuum_gripper_link_name']
        
        self._object_model_name = ""
        self._object_link_name = param_config_vacuum_gripper['attachable_object_link_name']
        
        self._attachable_object_prefix = param_config_vacuum_gripper['attachable_object_prefix']
        self._attachable_object_delimiter = param_config_vacuum_gripper['attachable_object_delimiter']
        self._logical_camera_topic_name = param_config_vacuum_gripper['logical_camera_topic_name']

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=0.1):
            box_model_name = self._object_model_name
            scene = ur5._scene

            start = rospy.get_time()
            seconds = rospy.get_time()
            while (seconds - start < timeout) and not rospy.is_shutdown():
                attached_objects = scene.get_attached_objects([box_model_name])
                self.is_attached = len(attached_objects.keys()) > 0

                is_known = box_model_name in scene.get_known_object_names()

                # Test if we are in the expected state
                if (box_is_attached == is_attached) and (box_is_known == is_known):
                    return True

                # Sleep so that we give other threads time on the processor
                rospy.sleep(0.1)
                seconds = rospy.get_time()

            # If we exited the while loop without returning then we timed out
            return False

    def conveyor_control(self, conveyor_speed):
        try:
            request = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
            conveyor_req = conveyorBeltPowerMsgRequest(conveyor_speed)
            return request(conveyor_req)
        except rospy.ServiceException:
            rospy.logerr ("Failed to start conveyor")

    def logical_camera_clbk(self, pkg_status):
        #get output from logical_camera_2 {rostopic echo /eyrc/vb/logical_camera_2}
        modelmsg = pkg_status.models
        conveyor_speed_default = 100
        self.conveyor_control(conveyor_speed_default)

        if len(modelmsg) != 0:
            rospy.sleep(.2)
            #  Fix the small .2 seconds delay...or add (stop the package when y>=0)
            conveyor_speed = 0
            self.conveyor_control(conveyor_speed)
            self.ee_move()
        
    

    def ur5_camera_clbk(self, pkg_type):
        number_models = len(pkg_type.models)

        for i in range(0, number_models):
            name_model = pkg_type.models[i].type
            
            lst_name_model = name_model.split(self._attachable_object_delimiter)
            
            if(lst_name_model[0] == self._attachable_object_prefix):
                #rospy.loginfo( '\033[94m' + " Package name: {}".format(name_model) + '\033[0m')
                self._object_model_name = name_model
                break

    def ee_move(self, box_is_attached=False):
        my_tf = tfEcho()
        box_id = self._object_model_name

        reference_frame1 = "logical_camera_2_frame"
        target_frame1 = "logical_camera_2_packagen1_frame"
        reference_frame2 = "world"
        target_frame2 = "ur5_ee_link"

        # fix the starting delay of the arm
        # fix arm's activation and deactivation problem and reduce time
        rospy.sleep(.5)

        #==================
        # Pose
        #==================
        pkg_world_tf = my_tf.func_tf(reference_frame2, target_frame1)
        x = (pkg_world_tf["px"])
        y = (pkg_world_tf["py"])
        z = (pkg_world_tf["pz"]) + 0.20
        
        pkg_pose = geometry_msgs.msg.Pose()
        pkg_pose.position.x = x
        pkg_pose.position.y = y
        pkg_pose.position.z = z
        # This to keep EE parallel to Ground Plane
        pkg_pose.orientation.x = -0.5
        pkg_pose.orientation.y = -0.5
        pkg_pose.orientation.z = 0.5
        pkg_pose.orientation.w = 0.5

        attach = "attach"
        detach = "detach"
        #fix make sure only after one is complete go to next
        ur5.go_to_pose(pkg_pose)
        self.attach_box()
        self.to_bin(box_id)
        self.detach_box()

        """
        print "_____________________"
        print ur5_ee_tf
        print box_model_name
        print "_____________________"
        """

    def attach_box(self, timeout=1):
        box_model_name = self._object_model_name

        ur5._scene.attach_box(ur5._eef_link, box_model_name, touch_links='vacuum_gripper_link')
        rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
        try:
            request = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
            print "Activating gripper"
            eef_req = vacuumGripperRequest(True)
            return request(eef_req)
        except rospy.ServiceException:
            rospy.logerr ("Failed to activate gripper")

        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=1):
        box_model_name = self._object_model_name

        ur5._scene.remove_attached_object(ur5._eef_link, name=box_model_name)
        rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
        try:
            request = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
            print "Deactivating gripper"
            eef_req = vacuumGripperRequest(False)
            return request(eef_req)
        except:
            rospy.logerr ("Failed to deactivate gripper")
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def to_bin(self, box_id):

        if box_id == "packagen1":
            self.generic_pose(0.11, 0.65, 1.30) #red bin pose

        if box_id == "packagen2":
            self.generic_pose(0.75, 0.03, 1.30) #green bin pose

        if box_id == "packagen3":
            self.generic_pose(0.4, -0.65, 1.30) #blue bin pose

    def generic_pose(self, px, py, pz):
        rospy.sleep(.2)

        generic_pose = geometry_msgs.msg.Pose()
        generic_pose.position.x = px
        generic_pose.position.y = py
        generic_pose.position.z = pz
        # This to keep EE parallel to Ground Plane
        generic_pose.orientation.x = -0.5
        generic_pose.orientation.y = -0.5
        generic_pose.orientation.z = 0.5
        generic_pose.orientation.w = 0.5
        ur5.go_to_pose(generic_pose)


def main():
    Manipulation = manipulation()
    while not rospy.is_shutdown():
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')

        rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, Manipulation.logical_camera_clbk)
        rospy.Subscriber("eyrc/vb/ur5_1/vacuum_gripper/logical_camera/ur5_1", LogicalCameraImage, Manipulation.ur5_camera_clbk)
        rospy.spin()    

if __name__ == '__main__':
    ur5 = CartesianPath()
    main()

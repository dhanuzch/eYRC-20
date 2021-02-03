#! /usr/bin/env python

import rospy

from hrwros_gazebo.msg import LogicalCameraImage

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def logi_cam_1(data):
    model_list = data.models
    no_of_models = len(model_list)

    if no_of_models == 0:
        #print(bcolors.OKCYAN + "lo_cam_1-no pkg present " + bcolors.ENDC)
        rospy.set_param("/master_view/lo_cam_1", "nopkg")
        pass

    if no_of_models >=1:
        for i in range(no_of_models):
            model = data.models[i].type
            if model != "ur5":
                rospy.set_param("/master_view/lo_cam_1", "pkgPresent")
            if model == "ur5":
                try:
                    model = data.models[i+1].type
                    rospy.set_param("/master_view/lo_cam_1", "pkgPresent")
                except:
                    print "No package found"

            print(bcolors.OKGREEN + "lo_cam_1-pkg present " + "(" + model + ")" +bcolors.ENDC)
            rospy.set_param("/master_view/lo_cam_1_pkg_name", model)

def logi_cam_2(data):
    model_list = data.models
    no_of_models = len(model_list)

    if no_of_models == 0:
        #print(bcolors.OKCYAN + "lo_cam_2-no pkg present " + bcolors.ENDC)
        rospy.set_param("/master_view/lo_cam_2", "nopkg")
        pass

    if no_of_models >=1:
        rospy.set_param("/master_view/lo_cam_2", "pkgPresent")
        for i in range(no_of_models):
            model = data.models[i].type
            
            if model == "ur5":
                try:
                    model = data.models[i+1].type
                except:
                    print "No package found"

            print(bcolors.OKGREEN + "lo_cam_2-pkg present " + "(" + model + ")" +bcolors.ENDC)
            rospy.set_param("/master_view/lo_cam_2_pkg_name", model)
            
def ur5_1_gripper(data):
    model_list = data.models
    no_of_models = len(model_list)

    if no_of_models == 0:
        #print(bcolors.OKCYAN + "ur5_1_gripper-no pkg present " + bcolors.ENDC)
        rospy.set_param("/master_view/ur5_1_gripper", "notActive")
        pass
    
    if no_of_models >=1:
        model = data.models[0].type
        rospy.set_param("/master_view/ur5_1_gripper", "Active")
        print(bcolors.OKGREEN + "ur5_1_gripper-pkg present " + "(" + model + ")" +bcolors.ENDC)
       

def ur5_2_gripper(data):
    model_list = data.models
    no_of_models = len(model_list)

    if no_of_models == 0:
        #print(bcolors.OKCYAN + "ur5_1_gripper-no pkg present " + bcolors.ENDC)
        rospy.set_param("/master_view/ur5_2_gripper", "notActive")
        pass
    
    if no_of_models >=1:
        model = data.models[0].type
        rospy.set_param("/master_view/ur5_2_gripper", "Active")
        print(bcolors.OKGREEN + "ur5_1_gripper-pkg present " + "(" + model + ")" +bcolors.ENDC)

def ur5_1_status(data):
    pass

def ur5_2_status(data):
    pass

def main():
    rospy.Subscriber("/eyrc/vb/logical_camera_1", LogicalCameraImage, logi_cam_1, queue_size=1)
    rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, logi_cam_2, queue_size=1)

    rospy.Subscriber("eyrc/vb/ur5_2/vacuum_gripper/logical_camera/ur5_2", LogicalCameraImage, ur5_2_gripper, queue_size=1)
    rospy.Subscriber("eyrc/vb/ur5_1/vacuum_gripper/logical_camera/ur5_1", LogicalCameraImage, ur5_1_gripper, queue_size=1)
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("node_master_view")
    main()
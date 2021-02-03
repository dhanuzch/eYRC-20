#! /usr/bin/env python

import rospy
import threading
import actionlib

from pkg_task5.msg import myActionMsgAction, myActionMsgFeedback, myActionMsgGoal, myActionMsgResult

def ur5_1_control():
    for i in range(10):
        print "ur5_1:", i, "\t"
        rospy.sleep(1)

def ur5_2_control():
    for i in range(10):
        print "ur5_2:", i, "\t"
        rospy.sleep(1)

def conveyor_control():
    pass

class ActionServer():

    def __init__(self):
        self.a_server = actionlib.SimpleActionServer(
            "/action_master_control", myActionMsgAction, execute_cb=self.execute_cb, auto_start=False)
        self.a_server.start()

    def execute_cb(self, goal):
            #TODO: This part
        success = True
        last_dish_washed = ''
        feedback = myActionMsgFeedback()
        result = myActionMsgResult()
        rate = rospy.Rate(1)

        for i in range(0, goal.number_of_minutes):
            if self.a_server.is_preempt_requested():
                self.a_server.set_preempted()
                success = False
                break

            last_dish_washed = 'bowl-' + str(i)
            feedback.last_dish_washed = last_dish_washed
            result.dishes_washed.append(last_dish_washed)
            self.a_server.publish_feedback(feedback)
            rate.sleep()

        if success:
            self.a_server.set_succeeded(result)
def main():
    ur5_1_thread = threading.Thread(name=ur5_1_control, target=ur5_1_control)
    ur5_2_thread = threading.Thread(name=ur5_2_control, target=ur5_2_control)

    ur5_1_thread.start()
    ur5_2_thread.start()


    rospy.spin() 

if __name__ == "__main__":
    rospy.init_node("node_controller")
    server = ActionServer()
    main()
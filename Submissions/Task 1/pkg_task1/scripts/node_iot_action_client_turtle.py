#!/usr/bin/env python
""" by @dhanuzch - ROS Node - Simple Action Client - Turtle and Action client Iot
This code can send goals(Distance and angle) to Turtle server and to ros_iot_bridge"""
import time
import rospy
import actionlib

from pkg_task1.msg import msgTurtleAction
from pkg_task1.msg import msgTurtleGoal
from pkg_task1.msg import msgTurtleActionResult

from pkg_ros_iot_bridge.msg import msgMqttSub
from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotGoal
from pkg_ros_iot_bridge.msg import msgRosIotResult

class IotRosBridgeActionClient:
    """This class is used to push goals to node_action_server_ros_iot_bridge"""
    # Constructor
    def __init__(self):

        # Initialize Action Client
        self._ac = actionlib.ActionClient('/action_ros_iot',
                                          msgRosIotAction)
        # Dictionary to Store all the goal handels
        self._goal_handles = {}

        # Store the MQTT Topic on which to Publish in a variable
        param_config_iot = rospy.get_param('config_iot')
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']

        # Wait for Action Server that will use the action - '/action_iot_ros' to start
        self._ac.wait_for_server()
        rospy.loginfo("Action server up, we can send goals.")

    def on_transition(self, goal_handle):
        """This function will be called when there is a change of state in the Action Client"""
        # from on_goal() to on_transition(). goal_handle generated by send_goal() is used here.
        result = msgRosIotResult()
        index = 0
        for i in self._goal_handles:
            if self._goal_handles[i] == goal_handle:
                index = i
                break

        rospy.loginfo("Transition Callback. Client Goal Handle #: " + str(index))
        rospy.loginfo("Comm. State: " + str(goal_handle.get_comm_state()))
        rospy.loginfo("Goal Status: " + str(goal_handle.get_goal_status()))
        # Comm State - Monitors the State Machine of the Client which is different from Server's
        # Comm State = 2 -> Active
        # Comm State = 3 -> Wating for Result
        # Comm State = 7 -> Done
        # if (Comm State == ACTIVE)
        if goal_handle.get_comm_state() == 2:
            rospy.loginfo(str(index) + ": Goal just went active.")
        # if (Comm State == DONE)
        if goal_handle.get_comm_state() == 7:
            rospy.loginfo(str(index) + ": Goal is DONE")
            rospy.loginfo(goal_handle.get_terminal_state())
            # get_result() gets the result produced by the Action Server
            result = goal_handle.get_result()
            rospy.loginfo(result.flag_success)

            if result.flag_success == True:
                rospy.loginfo("Goal successfully completed. Client Goal Handle #: " + str(index))
            else:
                rospy.loginfo("Goal failed. Client Goal Handle #: " + str(index))


    def send_goal(self, arg_protocol, arg_mode, arg_topic, arg_message):
        """This function is used to send Goals to Action Server"""
        # Create a Goal Message object
        goal = msgRosIotGoal()

        goal.protocol = arg_protocol
        goal.mode = arg_mode
        goal.topic = arg_topic
        goal.message = arg_message

        rospy.loginfo("Send goal.")
        # self.on_transition - It is a function pointer to a function which will be called when
        #                       there is a change of state in the Action Client State Machine
        goal_handle = self._ac.send_goal(goal,
                                         self.on_transition,
                                         None)

        return goal_handle

class SimpleActionClientTurtle:
    """This class can send goals to node_simple_actio_server_turtle"""

    # Constructor
    def __init__(self):
        self._ac = actionlib.SimpleActionClient('/action_turtle',
                                                msgTurtleAction)
        self._ac.wait_for_server()
        rospy.loginfo("Action server is up, we can send new goals!")

    def send_goal(self, arg_dis, arg_angle):
        """Function to send Goals to Action Servers"""
        # Create Goal message for Simple Action Server
        goal = msgTurtleGoal(distance=arg_dis, angle=arg_angle)

        self._ac.send_goal(goal, done_cb=self.done_callback,
                           feedback_cb=self.feedback_callback)
        rospy.loginfo("Goal has been sent.")

    def done_callback(self, status, result):
        """Function print result on Goal completion"""
        rospy.loginfo("Status is : " + str(status))
        rospy.loginfo("Result is : " + str(result))

    def feedback_callback(self, feedback):
        """Function to print feedback while Goal is being processed"""
        rospy.loginfo(feedback)

def callback_iotval(msg):
    """Turtle result is obtained and sent as a goal to node_action_server_ros_iot_bridge"""
    action_client = IotRosBridgeActionClient()

    value = msg.result
    x_res = value.final_x
    y_res = value.final_y
    theta = value.final_theta
    finaltuple = str((x_res, y_res, theta))
    sheetmsgdict = {"turtle_x":x_res, "turtle_y":y_res, "turtle_theta":theta}
    sheetmsg = str(sheetmsgdict)

    goal_handle1 = action_client.send_goal("http", "pub",
                                           action_client._config_mqtt_pub_topic, sheetmsg)
    action_client._goal_handles['1'] = goal_handle1
    goal_handle2 = action_client.send_goal("mqtt", "pub",
                                           action_client._config_mqtt_pub_topic, finaltuple)
    action_client._goal_handles['2'] = goal_handle2


def callback_mqttmsg(data):
    """"starts to send goal to node_simple_action_server_turtle,
    when 'start' is recieved from mqttbroker"""
    obj_client = SimpleActionClientTurtle()
    hexmsg = data.message

    if hexmsg == 'start':
        # Send Goals to Draw a Hexagon
        obj_client.send_goal(2, 0)
        rospy.sleep(10)

        obj_client.send_goal(2, 60)
        rospy.sleep(10)

        obj_client.send_goal(2, 60)
        rospy.sleep(10)

        obj_client.send_goal(2, 60)
        rospy.sleep(10)

        obj_client.send_goal(2, 60)
        rospy.sleep(10)

        obj_client.send_goal(2, 60)

def main():
    """Main function"""
    # Initialize ROS Node
    rospy.init_node('node_iot_action_client_turtle')
    rospy.Subscriber("/ros_iot_bridge/mqtt/sub", msgMqttSub, callback_mqttmsg)
    rospy.Subscriber("/action_turtle/result", msgTurtleActionResult, callback_iotval)
    # rospy.sleep(1.0)
    # goal_handle1.cancel()

    # Loop forever
    rospy.spin()


if __name__ == '__main__':
    main()

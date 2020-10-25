#!/usr/bin/env python
"""ROS Node - Action Server - IoT ROS Bridge by @dhanuzch"""
import threading
import requests
import rospy
import actionlib

from pkg_ros_iot_bridge.msg import msgRosIotAction      # Message that is used by ROS Actions
from pkg_ros_iot_bridge.msg import msgRosIotGoal        # Message that is used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotFeedback    # Message that is used for Feedback Msgs
from pkg_ros_iot_bridge.msg import msgRosIotResult      # Message that is used for Result Messages
from pkg_ros_iot_bridge.msg import msgMqttSub           # Message for MQTT Subscription Messages

from pyiot import iot                                   # Custom Python Module to perfrom MQTT Task

class IotRosBridgeActionServer:
    """This class can recieve goals and process it"""

    # Constructor
    def __init__(self):
        # Initialize the Action Server
        self._as = actionlib.ActionServer('/action_ros_iot',
                                          msgRosIotAction,
                                          self.on_goal,
                                          self.on_cancel,
                                          auto_start=False)

        '''
            * self.on_goal - It is the fuction pointer which points to a function which will be
                              called when the Action Server receives a Goal.

            * self.on_cancel - It is the fuction pointer which points to a function which will be
                               called when the Action Server receives a Cancel Request.
        '''

        # Read and Store IoT Configuration data from Parameter Server
        param_config_iot = rospy.get_param('config_iot')
        self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
        self._config_mqtt_sub_topic = param_config_iot['mqtt']['topic_sub']
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
        self._config_mqtt_qos = param_config_iot['mqtt']['qos']
        self._config_mqtt_sub_cb_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']
        print param_config_iot


        # Initialize ROS Topic Publication
        # Incoming msg from MQTT Sub will be published on a ROS Topic (/ros_iot_bridge/mqtt/sub)
        # ROS Nodes can sub to this ROS Topic /ros_iot_bridge/mqtt/sub to get msg from MQTT Sub
        self._handle_ros_pub = rospy.Publisher(self._config_mqtt_sub_cb_ros_topic,
                                               msgMqttSub, queue_size=10)


        # Sub to MQTT Topic (eyrc/xYzqLm/iot_to_ros) which is defined in 'config_iot_ros.yaml'
        # self.mqtt_sub_callback() function will be called when there is a msg from MQTT Sub
        ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                              self._config_mqtt_server_url,
                                              self._config_mqtt_server_port,
                                              self._config_mqtt_sub_topic,
                                              self._config_mqtt_qos)
        if ret == 0:
            rospy.loginfo("MQTT Subscribe Thread Started")
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread")


        # Start the Action Server
        self._as.start()

        rospy.loginfo("Started ROS-IoT Bridge Action Server.")

    def mqtt_sub_callback(self, client, userdata, message):
        """This is a callback function for MQTT Subscriptions"""
        payload = str(message.payload.decode("utf-8"))

        msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = message.topic
        msg_mqtt_sub.message = payload

        self._handle_ros_pub.publish(msg_mqtt_sub)

    def on_goal(self, goal_handle):
        """This function will be called when Action Server receives a Goal"""
        goal = goal_handle.get_goal()

        rospy.loginfo("Received new goal from Client")
        rospy.loginfo(goal)

        # Validate incoming goal parameters
        if (goal.protocol == "mqtt") or (goal.protocol == "http"):

            if (goal.mode == "pub") or (goal.mode == "sub"):
                goal_handle.set_accepted()

                # (For Asynchronous Processing of Goals)
                # Start a new thread to process new goal from the client
                # 'self.process_goal' - points to a function that will process incoming Goals
                thread = threading.Thread(name="worker",
                                          target=self.process_goal,
                                          args=(goal_handle,))
                thread.start()


            else:
                goal_handle.set_rejected()
                return
        else:
            goal_handle.set_rejected()
            return

    def process_goal(self, goal_handle):
        """This function is called is a separate thread to process Goal"""

        flag_success = False
        result = msgRosIotResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing goal : " + str(goal_id.id))

        goal = goal_handle.get_goal()

        # Goal Processing
        if goal.protocol == "mqtt":
            rospy.logwarn("MQTT")

            if goal.mode == "pub":
                rospy.logwarn("MQTT PUB Goal ID: " + str(goal_id.id))

                rospy.logwarn(goal.topic + " > " + goal.message)

                ret = iot.mqtt_publish(self._config_mqtt_server_url,
                                       self._config_mqtt_server_port,
                                       goal.topic,
                                       goal.message,
                                       self._config_mqtt_qos)

                if ret == 0:
                    rospy.loginfo("MQTT Publish Successful.")
                    result.flag_success = True
                else:
                    rospy.logerr("MQTT Failed to Publish")
                    result.flag_success = False

            elif goal.mode == "sub":
                rospy.logwarn("MQTT SUB Goal ID: " + str(goal_id.id))
                rospy.logwarn(goal.topic)

                ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                                      self._config_mqtt_server_url,
                                                      self._config_mqtt_server_port,
                                                      goal.topic,
                                                      self._config_mqtt_qos)
                if ret == 0:
                    rospy.loginfo("MQTT Subscribe Thread Started")
                    result.flag_success = True
                else:
                    rospy.logerr("Failed to start MQTT Subscribe Thread")
                    result.flag_success = False

        if goal.protocol == "http":
            rospy.logwarn("HTTP")

            if goal.mode == "pub":
                rospy.logwarn("HTTP PUB Goal ID: " + str(goal_id.id))

                rospy.logwarn(goal.topic + " > " + goal.message)


                p1dict = eval(goal.message)
                x_res = p1dict['turtle_x']
                y_res = p1dict['turtle_y']
                theta = p1dict['turtle_theta']

                parameters1 = {"id":"Sheet1", "turtle_x":x_res,
                               "turtle_y":y_res, "turtle_theta":theta}
                parameters2 = {"id":"task1", "team_id":"VB_2113", "unique_id":"UaHaYix",
                               "turtle_x":x_res, "turtle_y":y_res, "turtle_theta":theta}
                url1 = "https://script.google.com/macros/s/AKfycbzHbZ229Ab0TA91vdOiUQwJuc_BtBjTHg6_qK7xtLwey3iT6O7P/exec"
                url2 = "https://script.google.com/macros/s/AKfycbw850dk4moVgebU2GGe0PUQUvvg8jTpSjBQCawJt3_13vgujLk/exec"

                ret1 = requests.get(url1, params=parameters1)
                ret2 = requests.get(url2, params=parameters2)

                if (ret1.content == 'success') and (ret2.content == 'success'):
                    rospy.loginfo("HTTP Publish Successful.")
                    result.flag_success = True
                else:
                    rospy.logerr("HTTP Failed to Publish")
                    result.flag_success = False

            elif goal.mode == "sub":
                rospy.logwarn("HTTP SUB Goal ID: " + str(goal_id.id))
                rospy.logwarn(goal.topic)

                ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                                      self._config_mqtt_server_url,
                                                      self._config_mqtt_server_port,
                                                      goal.topic,
                                                      self._config_mqtt_qos)
                if ret == 0:
                    rospy.loginfo("HTTP Subscribe Thread Started")
                    result.flag_success = True
                else:
                    rospy.logerr("Failed to start HTTP Subscribe Thread")
                    result.flag_success = False


        rospy.loginfo("Send goal result to client")
        if result.flag_success == True:
            rospy.loginfo("Succeeded")
            goal_handle.set_succeeded(result)
        else:
            rospy.loginfo("Goal Failed. Aborting.")
            goal_handle.set_aborted(result)

        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")

    def on_cancel(self, goal_handle):
        """This function will be called when Goal Cancel request is send to the Action Server"""
        rospy.loginfo("Received cancel request.")
        goal_id = goal_handle.get_goal_id()


def main():
    """Main function"""
    rospy.init_node('node_action_server_ros_iot_bridge')

    action_server = IotRosBridgeActionServer()

    rospy.spin()

if __name__ == '__main__':
    main()

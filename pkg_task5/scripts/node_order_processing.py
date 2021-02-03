#!/usr/bin/env python
import rospy
from pkg_iot_ros_bridge.msg import msgMqttSub

import requests

priority_list = []
class orderProcessing():
    def __init__(self):
        param_config_iot = rospy.get_param('config_iot')
        self._web_app_url = param_config_iot['google_apps']['web_app_url']

    def mqtt_msg_sub_clbk(self, msg):
        msg = msg.message
        in_order_dict = eval(msg)

        city = in_order_dict["city"]
        order_time = in_order_dict["order_time"]
        order_id = in_order_dict["order_id"]
        lon = in_order_dict["lon"]
        lat = in_order_dict["lat"]
        qty = in_order_dict["qty"]
        item = in_order_dict["item"]

        if order_id.startswith("1"):
            priority = "HP"
            cost = "450"
        if order_id.startswith("2"):
            priority = "MP"
            cost = "250"
        if order_id.startswith("3"):
            priority = "LP"
            cost = "150"

        sheet_dict = {
            "id": "IncomingOrders",
            "Team Id": "VB#2113",
            "Unique Id": "UaHaYix",
            "Order ID": order_id,
            "Order Date and Time": order_time,
            "Item": item,
            "Priority": priority,
            "Order Quantity": qty,
            "City": city,
            "Longitude": lon,
            "Latitude": lat,
            "Cost": cost
        }

        self.order_sort(order_id)
        url = self._web_app_url

        ret1 = requests.get(url, params= sheet_dict)	
        ret = ret1.content
        print sheet_dict

    def order_sort(self, order_id):
        global priority_list


        if order_id.startswith("1"):
            priority_no = 1
        if order_id.startswith("2"):
            priority_no = 2
        if order_id.startswith("3"):
            priority_no = 3

        final_pri_list = priority_list.append(priority_no)

        if len(priority_list) >= 2:
            priority_list.sort()

        print(priority_list)

def main():
    rospy.init_node('node_order_processing')
    rospy.Subscriber("/ros_iot_bridge/mqtt/sub", msgMqttSub, processing.mqtt_msg_sub_clbk)

    rospy.spin()


if __name__ == '__main__':
    processing = orderProcessing()
    main()

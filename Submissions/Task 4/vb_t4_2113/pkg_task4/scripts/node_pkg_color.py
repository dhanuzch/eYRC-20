#!/usr/bin/env python

import rospy

from pkg_task4.msg import conveyor

def conveyor_msg(msg):
    #print (msg)
    pkg_id = msg.pkg_id
    #print (pkg_id)
    pkg_name(pkg_id)


def pkg_name(packagen_name):
    pkg_details = rospy.get_param("/pkg_details")
    #print (pkg_details)
    pkg_dict = next((item for item in pkg_details if item["pkg_id"] == packagen_name), False)
    #pkg_dict = next(item for item in pkg_details if item["pkg_id"] == packagen_name)

    print (pkg_dict["color"])

def main():
    rospy.Subscriber("/eyrc/conveyor_msg", conveyor, conveyor_msg)
    rospy.spin()    

if __name__ == '__main__':
    rospy.init_node('node_pkg_color', anonymous=True)
    main()

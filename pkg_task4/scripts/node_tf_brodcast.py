#!/usr/bin/env python  
import rospy

import tf2_ros
import tf2_msgs.msg


class tfEcho:

    def __init__(self):
        rospy.init_node('node_tf_echo')
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)


    def func_tf(self, arg_frame_1, arg_frame_2):
        try:
            trans = self._tfBuffer.lookup_transform(arg_frame_1, arg_frame_2, rospy.Time(0))
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



def main():
    my_tf = tfEcho()

    reference_frame = "world"
    target_frame = "ur5_wrist_3_link"

    while not rospy.is_shutdown():
        my_tf.func_tf_print(reference_frame, target_frame)
        rospy.sleep(1)

    del my_tf


if __name__ == '__main__':
    main()

#!/usr/bin/env python

import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('gripper_tf_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(1.0)
    rospy.sleep(0.1)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/gripper_link', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        print trans, rot
        # rospy.logerr("Angular: ", rot[0], rot[1], rot[2], rot[3])
        # rospy.logerr("linear: ", trans[0], trans[1], trans[2])

        rate.sleep()

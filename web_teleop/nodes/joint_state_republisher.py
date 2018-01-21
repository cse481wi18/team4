#!/usr/bin/env python

import fetch_api
import rospy
from joint_state_reader import JointStateReader
from std_msgs.msg import Float64

ARMS = fetch_api.ArmJoints.names()


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('joint_state_republisher')
    wait_for_time()
    torso_pub = rospy.Publisher('joint_state_republisher/torso_lift_joint',
                                Float64)
    reader = JointStateReader()
    rospy.sleep(0.5)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        val = reader.get_joint("torso_lift_joint")
        torso_pub.publish(val)
        arm_vals = reader.get_joints(ARMS)
        # publish arm_vals (make an arm_val publisher)

        rate.sleep()


if __name__ == '__main__':
    main()

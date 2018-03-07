#! /usr/bin/env python

import fetch_api
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('gripper_reader_demo')
    wait_for_time()
    rospy.sleep(0.5)
    names = fetch_api.GripperJoints()


if __name__ == '__main__':
    main()

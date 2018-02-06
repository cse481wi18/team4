#! /usr/bin/env python

import fetch_api
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('video5_1_demo')
    wait_for_time()
    PRE_GRASP_POSE = [1.49369115856, -0.009455542748545, -1.6215686602, 1.75733232639, -0.308369926759, -0.167083674359,
                      1.86430164807]
    GRASP_POSE = [0.309518132559, 0.824826994611, -2.83255981424, 0.542284385549, 0.274975280469, 0.337509054192,
                  2.67265523319]
    LIFT_POSE = [0.634045131397, 0.279614454339, -2.12515381918, 1.06190065588, -0.343333531167, -0.396500877504,
                 2.49131212711]

    arm = fetch_api.Arm()

    gripper = fetch_api.Gripper()
    effort = gripper.MAX_EFFORT

    arm.move_to_joints(fetch_api.ArmJoints.from_list(PRE_GRASP_POSE))
    rospy.sleep(3)
    gripper.open()
    arm.move_to_joints(fetch_api.ArmJoints.from_list(GRASP_POSE))
    rospy.sleep(3)
    gripper.close(effort)
    arm.move_to_joints(fetch_api.ArmJoints.from_list(LIFT_POSE))
    rospy.sleep(8)


if __name__ == '__main__':
    main()

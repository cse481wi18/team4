#!/usr/bin/env python

import fetch_api
import rospy
from web_teleop.srv import SetArm, SetArmResponse, SetGrip, SetGripResponse, SetHead, SetHeadResponse, SetTorso, \
    SetTorsoResponse


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ActuatorServer(object):
    def __init__(self):
        self._torso = fetch_api.Torso()
        self._arm = fetch_api.Arm()
        self._gripper = fetch_api.Gripper()
        self._head = fetch_api.Head()

    def handle_set_torso(self, request):
        self._torso.set_height(request.height)
        return SetTorsoResponse()

    def handle_set_arm(self, request):
        self._arm.move_to_joints(fetch_api.ArmJoints.from_list(request.positions))
        return SetArmResponse()

    def handle_set_grip(self, request):
        if request.grip:
            self._gripper.close()
        else:
            self._gripper.open()
        return SetGripResponse()

    def handle_set_head(self, request):
        self._head.pan_tilt(0.0, request.tilt)
        return SetHeadResponse()


def main():
    rospy.init_node('web_teleop_actuators')
    wait_for_time()
    server = ActuatorServer()
    torso_service = rospy.Service('web_teleop/set_torso', SetTorso,
                                  server.handle_set_torso)
    arm_service = rospy.Service('web_teleop/set_arm', SetArm,
                                server.handle_set_arm)
    head_service = rospy.Service('web_teleop/set_head', SetHead,
                                 server.handle_set_head)
    grip_service = rospy.Service('web_teleop/set_grip', SetGrip,
                                 server.handle_set_grip)
    rospy.spin()


if __name__ == '__main__':
    main()

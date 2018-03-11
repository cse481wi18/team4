#!/usr/bin/env python

import fetch_api
import rospy
from web_teleop.srv import SetArm, SetArmResponse, SetGrip, SetGripResponse, SetHead, SetHeadResponse, SetTorso, \
    SetTorsoResponse

import wait_for_time
import brain


class ActuatorServer(object):
    def __init__(self):
        self._brain = None

    def handle_start(self, request):
        self._brain = brain.Brain()
        self._brain
        return True

    def handle_stop(self, request):
        del self._brain
        return True



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

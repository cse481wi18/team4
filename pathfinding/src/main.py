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
        self._brain.run()
        return True

    def handle_stop(self, request):
        del self._brain # TODO ???
        self._brain = None
        return True



def main():
    rospy.init_node('web_teleop_actuators')
    wait_for_time()
    server = ActuatorServer()
    start_bot_service = rospy.Service('start_ballbot_topic', SetTorso,
                                  server.handle_set_torso)
    stop_bot_service = rospy.Service('stop_ballbot_topic', SetTorso,
                                  server.handle_stop)
    rospy.spin()


if __name__ == '__main__':
    main()

#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty

import wait_for_time
import brain


class Server(object):
    def __init__(self):
        self._brain = None

    def handle_start(self, request):
        rospy.logerr("Called handle_start")
        print "Called handle_start"
        self._brain = brain.Brain()
        self._brain.run()
        return True

    def handle_stop(self, request):
        del self._brain # TODO ???
        self._brain = None
        return True

def main():
    rospy.init_node('Server')
    wait_for_time.wait_for_time()
    print "started webteleoajfsodj;igodsaigz"
    server = Server()
    rospy.Subscriber('start_ballbot_topic', Empty, callback=server.handle_start)
    # stop_bot_service = rospy.Service('stop_ballbot_topic', String,
    #                               server.handle_stop)
    rospy.spin()


if __name__ == '__main__':
    main()

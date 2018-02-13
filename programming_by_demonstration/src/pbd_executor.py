#! /usr/bin/env python

import sys, os, time
import rospy
import fetch_api
from pbd_recorder import Recorder


class Interface:
    def __init__(self):
        self.recorder = Recorder()

    def run(self):
        while True:
            input = raw_input("enter path to execute >> ")
            self.recorder.execute_path(input)


def main():
    rospy.init_node('pbd_executor')
    wait_for_time()
    torso = fetch_api.Torso()
    print 'Raising torso height...'
    torso.set_height(torso.MAX_HEIGHT)
    demo_runner = Interface()
    demo_runner.run()


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


# Main Program
if __name__ == '__main__':
    main()

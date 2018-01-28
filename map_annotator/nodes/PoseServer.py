#!/usr/bin/env python

import rospy
from map_annotator.msg import PoseNames


class PoseServer(object):
    def __init__(self):
        self.PoseList = {}

        name_publisher = rospy.Publisher('/map_annotator', PoseNames, queue_size=10)

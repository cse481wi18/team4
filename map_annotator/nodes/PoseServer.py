#!/usr/bin/env python

import rospy
import copy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from map_annotator.msg import PoseNames, UserAction
from geometry_msgs.msg import PoseStamped


class PoseServer(object):

    def handleAction(self, msg):
        command = msg.command

        if command == UserAction.CREATE:
            new_marker = InteractiveMarker();

            new_marker.header.frame_id = "map"
            new_marker.name = msg.name
            new_marker.description = msg.name
            new_marker.pose.position.z = 0.1

            arrow_marker = Marker()
            arrow_marker.type = Marker.ARROW
            arrow_marker.scale.x = 0.45
            arrow_marker.scale.y = 0.45
            arrow_marker.scale.z = 0.45
            arrow_marker.color.r = 0.0
            arrow_marker.color.g = 1.0
            arrow_marker.color.b = 0.0
            arrow_marker.color.a = 1.0

            pose_marker_control = InteractiveMarkerControl()
            pose_marker_control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
            pose_marker_control.always_visible = True
            pose_marker_control.markers.append(arrow_marker)
            new_marker.controls.append(pose_marker_control)

            self.marker_server.insert(new_marker)
            self.marker_server.applyChanges()

            # Copy? Will this update? How does the front end update position?
            self.PoseList[msg.name] = new_marker.pose

            self.name_publisher.Publish(self.PoseList.keys())

        elif command == UserAction.DELETE:
            # Breaks if msg.name does not exist in list.
            del self.PoseList[msg.name]

            self.marker_server.erase(msg.name)
            self.marker_server.applyChanges()

        elif command == UserAction.GOTO:
            if msg.name in self.PoseList.keys():
                newGoal = PoseStamped()
                newGoal.header.frame_id = "map"
                newGoal.pose = copy.deepcopy(self.PoseList[msg.name])

                self.goto_publisher.publish(newGoal)
            else:
                print "ERROR: No such pose", msg.name

    def __init__(self):
        self.PoseList = {}

        self.marker_server = InteractiveMarkerServer("/map_annotator/map_poses")

        self.name_publisher = rospy.Publisher('/map_annotator/pose_names', PoseNames, queue_size=10, latch=True)
        self.goto_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.action_subscriber = rospy.Subscriber('/map_annotator/user_actions', UserAction, callback=self.handleAction)


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('pose_server')
    wait_for_time()

    server = PoseServer()

    rospy.spin()


if __name__ == '__main__':
    main()

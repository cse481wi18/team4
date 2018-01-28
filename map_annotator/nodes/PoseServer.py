#!/usr/bin/env python

import rospy
import copy
import pickle
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from map_annotator.msg import PoseNames, UserAction
from geometry_msgs.msg import PoseStamped


class PoseServer(object):
    FILE_NAME = "saved_poses.p"

    def savePoses(self):
        rospy.logerr("Writing pickle file")
        try:
            pickle.dump(self.PoseList, open("saved_poses.p", "wb"))
            rospy.logerr("Dumped pickle file!")
        except Exception as e:
            rospy.roserr(e)

    def updatePosition(self, input):
        if (input.event_type == InteractiveMarkerFeedback.POSE_UPDATE):
            self.PoseList[input.marker_name] = copy.deepcopy(input.pose)
            self.savePoses()

    def createMarker(self, name, pose):
        new_marker = InteractiveMarker();

        new_marker.header.frame_id = "map"
        new_marker.name = name
        new_marker.description = name
        if pose is None:
            new_marker.pose.position.x = 0.0
            new_marker.pose.position.y = 0.0
            new_marker.pose.position.z = 0.1
            new_marker.pose.orientation.x = 0.0
            new_marker.pose.orientation.y = 0.0
            new_marker.pose.orientation.z = 0.0
            new_marker.pose.orientation.w = 1.0
        else:
            new_marker.pose = pose

        arrow_marker = Marker()
        arrow_marker.type = Marker.ARROW
        arrow_marker.scale.x = 0.40
        arrow_marker.scale.y = 0.05
        arrow_marker.scale.z = 0.05
        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 1.0
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 1.0

        pose_marker_control = InteractiveMarkerControl()
        pose_marker_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        pose_marker_control.orientation.w = 1;
        pose_marker_control.orientation.x = 0;
        pose_marker_control.orientation.y = 1;
        pose_marker_control.orientation.z = 0;
        pose_marker_control.always_visible = True
        pose_marker_control.markers.append(arrow_marker)
        new_marker.controls.append(pose_marker_control)

        pose_rotation_control = InteractiveMarkerControl()
        pose_rotation_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        pose_rotation_control.orientation.w = 1;
        pose_rotation_control.orientation.x = 0;
        pose_rotation_control.orientation.y = 1;
        pose_rotation_control.orientation.z = 0;
        pose_rotation_control.always_visible = True
        # pose_rotation_control.markers.append(arrow_marker)
        new_marker.controls.append(pose_rotation_control)

        self.marker_server.insert(new_marker, self.updatePosition)
        self.marker_server.applyChanges()

        # Copy? Will this update? How does the front end update position?
        self.PoseList[name] = copy.deepcopy(new_marker.pose)
        self.savePoses()

    def handleAction(self, msg):
        command = msg.command
        rospy.logerr("Handling action")

        if command == UserAction.CREATE:

            self.createMarker(msg.name, None)

            self.name_publisher.publish(self.PoseList.keys())

            # rospy.logerr(self.PoseList.keys())

        elif command == UserAction.DELETE:
            # Breaks if msg.name does not exist in list.
            del self.PoseList[msg.name]
            self.savePoses()

            self.marker_server.erase(msg.name)
            self.marker_server.applyChanges()

            self.name_publisher.publish(self.PoseList.keys())

        elif command == UserAction.GOTO:
            rospy.logerr("ExecutinupdatePositiong goto")
            if msg.name in self.PoseList.keys():
                newGoal = PoseStamped()
                newGoal.header.frame_id = "map"
                newGoal.pose = copy.deepcopy(self.PoseList[msg.name])

                newGoal.pose.position.z -= .1
                rospy.logerr(newGoal.pose)

                self.goto_publisher.publish(newGoal)
            else:
                print "ERROR: No such pose", msg.name

    def __init__(self):
        try:
            self.PoseList = pickle.load(open("saved_poses.p", "rb"))
            rospy.logerr("Read from pickle file.")
        except Exception as e:
            print e
            self.PoseList = {}

        self.marker_server = InteractiveMarkerServer("/map_annotator/map_poses")

        self.name_publisher = rospy.Publisher('/map_annotator/pose_names', PoseNames, queue_size=10, latch=True)
        self.goto_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.action_subscriber = rospy.Subscriber('/map_annotator/user_actions', UserAction, callback=self.handleAction)

        for name, pose in self.PoseList.items():
            self.createMarker(name, pose)

        self.name_publisher.publish(self.PoseList.keys())


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('map_annotator_server')
    wait_for_time()

    server = PoseServer()

    rospy.spin()


if __name__ == '__main__':
    main()

#!/usr/bin/env python

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
import fetch_api
import math
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

base = fetch_api.Base()

def handle_viz_input(input):
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        rospy.loginfo(input.marker_name + ' was clicked.')
    else:
        rospy.loginfo('Cannot handle this InteractiveMarker event')

def go_forward(input):
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        rospy.loginfo("going forward")
        base.go_forward(0.5)

def go_backward(input):
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        rospy.loginfo("going backward")
        base.go_forward(-0.5)

def turn_port(input):
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        rospy.loginfo("turning port")
        base.turn(45 * math.pi / 180)

def turn_starboard(input):
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        rospy.loginfo("turning starboard")
        base.turn(-45 * math.pi / 180)

def main():
    rospy.init_node('nav_node')
    # wait_for_time()
    rospy.sleep(0.5)
    # Create server
    server = InteractiveMarkerServer("simple_marker")
    # Create interactive marker
    fwd_marker = InteractiveMarker()
    fwd_marker.header.frame_id = "base_link"
    fwd_marker.name = "fwd_marker"
    fwd_marker.description = "Go forward"
    fwd_marker.pose.position.x = 1
    fwd_marker.pose.orientation.w = 1
    # create teal cube marker for the interactive marker
    fbox_marker = Marker()
    fbox_marker.type = Marker.CUBE
    fbox_marker.pose.orientation.w = 1
    fbox_marker.scale.x = 0.45
    fbox_marker.scale.y = 0.45
    fbox_marker.scale.z = 0.45
    fbox_marker.color.r = 0.5
    fbox_marker.color.g = 0.5
    fbox_marker.color.b = 0.0
    fbox_marker.color.a = 1.0

    bkw_marker = InteractiveMarker()
    bkw_marker.header.frame_id = "base_link"
    bkw_marker.name = "bkw_marker"
    bkw_marker.description = "Go backward"
    bkw_marker.pose.position.x = -1
    bkw_marker.pose.orientation.w = 1
    # create teal cube marker for the interactive marker
    bbox_marker = Marker()
    bbox_marker.type = Marker.CUBE
    bbox_marker.pose.orientation.w = 1
    bbox_marker.scale.x = 0.45
    bbox_marker.scale.y = 0.45
    bbox_marker.scale.z = 0.45
    bbox_marker.color.r = 0.5
    bbox_marker.color.g = 0.0
    bbox_marker.color.b = 0.5
    bbox_marker.color.a = 1.0

    prt_marker = InteractiveMarker()
    prt_marker.header.frame_id = "base_link"
    prt_marker.name = "prt_marker"
    prt_marker.description = "Turn port"
    prt_marker.pose.position.y = 1
    prt_marker.pose.orientation.w = 1
    # create teal cube marker for the interactive marker
    pbox_marker = Marker()
    pbox_marker.type = Marker.CUBE
    pbox_marker.pose.orientation.w = 1
    pbox_marker.scale.x = 0.45
    pbox_marker.scale.y = 0.45
    pbox_marker.scale.z = 0.45
    pbox_marker.color.r = 1.0
    pbox_marker.color.g = 0.0
    pbox_marker.color.b = 0.0
    pbox_marker.color.a = 1.0

    stb_marker = InteractiveMarker()
    stb_marker.header.frame_id = "base_link"
    stb_marker.name = "stb_marker"
    stb_marker.description = "Turn starboard"
    stb_marker.pose.position.y = -1
    stb_marker.pose.orientation.w = 1
    # create teal cube marker for the interactive marker
    sbox_marker = Marker()
    sbox_marker.type = Marker.CUBE
    sbox_marker.pose.orientation.w = 1
    sbox_marker.scale.x = 0.45
    sbox_marker.scale.y = 0.45
    sbox_marker.scale.z = 0.45
    sbox_marker.color.r = 0.0
    sbox_marker.color.g = 1.0
    sbox_marker.color.b = 0.0
    sbox_marker.color.a = 1.0

    # create an InteractiveMarkerControl, add the Marker to it, and
    fbutton_control = InteractiveMarkerControl()
    # add the control to the InteractiveMarker.
    fbutton_control.interaction_mode = InteractiveMarkerControl.BUTTON
    fbutton_control.always_visible = True
    fbutton_control.markers.append(fbox_marker)
    fwd_marker.controls.append(fbutton_control)

    bbutton_control = InteractiveMarkerControl()
    bbutton_control.interaction_mode = InteractiveMarkerControl.BUTTON
    bbutton_control.always_visible = True
    bbutton_control.markers.append(bbox_marker)
    bkw_marker.controls.append(bbutton_control)

    pbutton_control = InteractiveMarkerControl()
    pbutton_control.interaction_mode = InteractiveMarkerControl.BUTTON
    pbutton_control.always_visible = True
    pbutton_control.markers.append(pbox_marker)
    prt_marker.controls.append(pbutton_control)

    sbutton_control = InteractiveMarkerControl()
    sbutton_control.interaction_mode = InteractiveMarkerControl.BUTTON
    sbutton_control.always_visible = True
    sbutton_control.markers.append(sbox_marker)
    stb_marker.controls.append(sbutton_control)

    server.insert(fwd_marker, go_forward)
    server.insert(bkw_marker, go_backward)
    server.insert(stb_marker, turn_starboard)
    server.insert(prt_marker, turn_port)
    server.applyChanges()
    rospy.spin()


if __name__ == '__main__':
    main()

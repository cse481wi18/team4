#! /usr/bin/env python

import rospy
import map_driver
import ball_driver
import arm_controller
import perceptor
import wait_for_time
import fetch_api
import pickle
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, Header

# Note: Brain handles all conversions
# milestone 1 - no backpack

BASKET_POSITION = Pose() # TODO figure out map stuff for hallway

# read in from pickle
ROAM_POSITIONS = []

TIME_TO_PERCEIVE_BALL = 5
TORSO_HEIGHT_TO_PICKUP_BALL = 0.03
TORSO_HEIGHT_TO_UNFURL_ARM = 0.25

# TODO milestone 1
# find location behind target so that position that robot drives to is
#  in arms reach of ball/basket

marker_publisher = rospy.Publisher('happy_marker', Marker, queue_size=10)
cur_id = 0

def pub_pose(target):
    global cur_id
    marker = Marker(
        type=Marker.ARROW,
        pose=target,
        scale=Vector3(0.1, 0.1, 0.1),
        color=ColorRGBA(1.0, 0.0, 0.0, 0.5),
        header=Header(frame_id='base_link'),
        id=cur_id,
        lifetime=rospy.Duration(15)
    )
    cur_id += 1
    marker_publisher.publish(marker)

# TODO milestone one cancel all goals on ctrl-c

def load_annotated_positions():
    global BASKET_POSITION, ROAM_POSITIONS
    try:
        saved_poses = pickle.load(open("savedPoses.p", "rb"))
        BASKET_POSITION = saved_poses["basket"]
        del saved_poses["basket"]
        ROAM_POSITIONS = dict(saved_poses).values()
    except Exception as e:
        print "Couldn't read in annotated positions!, ", e
        return False
    return True

def main():
    rospy.init_node('brain')
    wait_for_time.wait_for_time()

    # read in roaming positions
    if not load_annotated_positions():
        exit(1)

    my_map_driver = map_driver.Driver()
    my_ball_driver = ball_driver.Driver()

    my_torso = fetch_api.Torso()
    my_perceptor = perceptor.Perceptor()
    my_arm = arm_controller.ArmController()
    my_head = fetch_api.Head()

    # raise torso before unfurling arm
    my_torso.set_height(TORSO_HEIGHT_TO_UNFURL_ARM)
    my_arm.tuck_arm()

    curr_roam_ind = 0
    while True:
        print "setting torso to maximum ball pickup position..."
        my_torso.set_height(TORSO_HEIGHT_TO_PICKUP_BALL)
        print "moving head to maximum ball finding position..."
        my_head.pan_tilt(0, 0.9)
        rospy.sleep(TIME_TO_PERCEIVE_BALL)
        ball_position = my_perceptor.get_closest_ball_location() # from perceptor node
        # pub_pose(ball_position)
        if ball_position is not None:
            print "Ball Found!"
            # Check if ball is reachable (within .5)
            if not my_arm.ball_reachable(ball_position):
                print "Ball is not reachable D:"
                my_ball_driver.go_to(ball_position)

                print "arrived at the ball? checking..."
            for i in range(3):
                print "moving head to maximum ball finding position..."
                my_head.pan_tilt(0, 0.9)
                rospy.sleep(TIME_TO_PERCEIVE_BALL)
                ball_position = my_perceptor.get_closest_ball_location()
                if ball_position is None:
                    print "We lost the ball! :'(((((((("
                    rospy.sleep(1)
                else:
                    break
            success = False
            if ball_position is not None:
                success = my_arm.pick_up_ball(ball_position)
            # assume for milestone 1 that basket is marked on map
            print "Picked up ball?, ", success
            if success:
                # driver.go_to(BASKET_POSITION)
                my_arm.drop_ball_in_basket()
            # driver.return_to_default_position()
        else:
            print "No ball found!"
            # if len(ROAM_POSITIONS) is not 0:
            #     print "roaming..."
            #     map_driver.go_to(ROAM_POSITIONS[curr_roam_ind])
            #     curr_roam_ind += 1
            #     curr_roam_ind = curr_roam_ind % len(ROAM_POSITIONS)
            # TODO milestone 2: move head if no ball seen
            # TODO milestone 3: move base if no ball seen
        rospy.sleep(1)



if __name__ == '__main__':
    main()

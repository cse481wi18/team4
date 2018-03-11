#! /usr/bin/env python

import rospy
import map_driver
import ball_driver
import arm_controller
import perceptor
import wait_for_time
import fetch_api
import webapp_handler
import pickle
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, Header

# Note: Brain handles all conversions
# milestone 1 - no backpack

POSITION_FILE_NAMES = "tennis_ball_robot_positions.py"
BASKET_POSITION = None # TODO figure out map stuff for hallway

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
        saved_poses = pickle.load(open(POSITION_FILE_NAMES, "rb"))
        BASKET_POSITION = saved_poses["basket"]
        del saved_poses["basket"]
        ROAM_POSITIONS = dict(saved_poses).values()
    except Exception as e:
        print "Couldn't read in annotated positions!, ", e
        return False
    return BASKET_POSITION is not None and ROAM_POSITIONS != []

class Brain(object):
    def run(self):
        rospy.init_node('brain')
        wait_for_time.wait_for_time()

        # read in roaming positions
        if not load_annotated_positions():
            exit(1)

        # register web app stuff
        server = webapp_handler.BallBotServer()
        start_service = rospy.Service('start_ballbot_topic', Empty,
                                         server.handle_start)

        my_map_driver = map_driver.Driver()
        my_ball_driver = ball_driver.Driver()

        my_torso = fetch_api.Torso()
        my_perceptor = perceptor.Perceptor()
        my_arm = arm_controller.ArmController()
        my_head = fetch_api.Head()

        # raise torso before unfurling arm
        print "[brain: unfurling arm]"
        my_torso.set_height(TORSO_HEIGHT_TO_UNFURL_ARM)
        rospy.sleep(5)
        my_arm.tuck_arm()
        print "[brain: setting torso to maximum ball pickup position...]"
        my_torso.set_height(TORSO_HEIGHT_TO_PICKUP_BALL)

        curr_roam_ind = 0
        while True:

            print "[brain: moving head to maximum ball finding position...]"
            my_head.pan_tilt(0, 0.9)
            rospy.sleep(TIME_TO_PERCEIVE_BALL)
            ball_position = my_perceptor.get_closest_ball_location() # from perceptor node
            # pub_pose(ball_position)
            if ball_position is not None:
                print "[brain: ball found]"
                if not my_arm.ball_reachable(ball_position):
                    print "[brain: ball is not reachable]"
                    # print "ball_position: "
                    # print ball_position
                    print "[brain: moving to ball...]"
                    pub_pose(ball_position)
                    my_ball_driver.go_to(ball_position)

                for i in range(3):
                    rospy.sleep(TIME_TO_PERCEIVE_BALL) # TODO sleep longer?
                    ball_position = my_perceptor.get_closest_ball_location()
                    if ball_position is None:
                        print "[brain: cannot find ball]"
                        rospy.sleep(1)
                    else:
                        break
                # TODO handle case of bumped ball
                if ball_position is not None:
                    success = False
                    if ball_position is not None:
                        print "[brain: picking up ball]"
                        success = my_arm.pick_up_ball(ball_position)
                    # assume for milestone 1 that basket is marked on map
                    if success:
                        print "[brain: pick successful, dropping ball]"
                        # driver.go_to(BASKET_POSITION)
                        my_arm.drop_ball_in_basket()
                        my_arm.tuck_arm()
                    else:
                        print "[brain: pick failed]"
                # driver.return_to_default_position()
            else:
                print "[brain: no ball found]"
                if len(ROAM_POSITIONS) is not 0:
                    print "roaming..."
                    map_driver.go_to(ROAM_POSITIONS[curr_roam_ind])
                    curr_roam_ind += 1
                    curr_roam_ind = curr_roam_ind % len(ROAM_POSITIONS)
                # TODO milestone 2: move head if no ball seen
                # TODO milestone 3: move base if no ball seen
            print "[brain: looping]"
            print "\n*************************************************************\n"
            rospy.sleep(1)


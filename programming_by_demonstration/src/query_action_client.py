from robot_controllers_msgs.msg import QueryControllerStatesAction, ControllerState, QueryControllerStatesGoal
import actionlib


class arm_control:
    def __init__(self):
        self._controller_client = actionlib.SimpleActionClient('query_controller_states',
                                                               QueryControllerStatesAction)

    def stop_arm(self):
        goal = QueryControllerStatesGoal()
        state = ControllerState()
        state.name = 'arm_controller/follow_joint_trajectory'
        state.state = ControllerState.STOPPED
        goal.updates.append(state)
        self._controller_client.send_goal(goal)
        self._controller_client.wait_for_result()

    def start_arm(self):
        goal = QueryControllerStatesGoal()
        state = ControllerState()
        state.name = 'arm_controller/follow_joint_trajectory'
        state.state = ControllerState.RUNNING
        goal.updates.append(state)
        self._controller_client.send_goal(goal)
        self._controller_client.wait_for_result()

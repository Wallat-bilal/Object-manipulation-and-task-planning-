#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, MoveItErrorCodes

class MoveToJointDynState(EventState):
    '''
    This state moves the robot to a specified joint configuration using MoveIt.

    -- move_group       string   Name of the MoveIt move group.

    #> target_joints    float[]  Target joint configuration.

    <= reached          The robot reached the target joint configuration.
    <= control_failed   The robot failed to execute the motion control.
    <= planning_failed  The planning of the motion failed.
    '''

    def __init__(self, move_group):
        super(MoveToJointDynState, self).__init__(outcomes=['reached', 'control_failed', 'planning_failed'],
                                                  input_keys=['target_joints'])

        self._topic = 'move_group'
        self._client = ProxyActionClient({self._topic: MoveGroupAction})  # proxy client to the move_group action server

    def execute(self, userdata):
        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)
            # Check the type of failure based on MoveIt's error codes
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return 'reached'
            elif result.error_code.val in [MoveItErrorCodes.CONTROL_FAILED]:
                Logger.logwarn('Control failed while executing move.')
                return 'control_failed'
            else:
                Logger.logwarn('Planning failed for the given joint configuration.')
                return 'planning_failed'

    def on_enter(self, userdata):
        goal = MoveGroupGoal()
        goal.request.group_name = self.move_group
        goal.request.target_joint_values = userdata.target_joints
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            Logger.logwarn('Failed to send goal to Move Group:\n%s' % str(e))
            self._client.cancel_all_goals()
            return 'planning_failed'


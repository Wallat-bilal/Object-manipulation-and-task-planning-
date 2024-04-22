#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from std_srvs.srv import SetBool, SetBoolRequest

class ToolControlState(EventState):
    '''
    This state controls the opening and closing of a tool such as a gripper.
    The service is expected to be of type std_srvs/SetBool, where true opens the tool and false closes it.

    -- tool_control_service string  Name of the ROS service to control the tool.

    #> control_command    bool  Command to send to the tool (True for open, False for close).

    <= success           The tool has successfully executed the command.
    <= failed            The tool control command could not be executed.
    '''

    def __init__(self, tool_control_service):
        super(ToolControlState, self).__init__(outcomes=['success', 'failed'],
                                               input_keys=['control_command'])
        self._service_name = tool_control_service
        self._client = ProxyServiceCaller({self._service_name: SetBool})

    def on_enter(self, userdata):
        service_request = SetBoolRequest()
        service_request.data = userdata.control_command
        try:
            result = self._client.call(self._service_name, service_request)
            if not result.success:
                Logger.logwarn('Tool control command failed.')
                return 'failed'
        except Exception as e:
            Logger.logwarn('Failed to call tool control service:\n%s' % str(e))
            return 'failed'
        return 'success'


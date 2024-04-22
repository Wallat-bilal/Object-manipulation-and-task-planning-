#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool, SetBoolResponse

def handle_tool_control(req):
    rospy.loginfo("Gripper open request: %s" % req.data)
    return SetBoolResponse(True)  # Simulate always succeeding

def tool_control_server():
    rospy.init_node('tool_control_server')
    s = rospy.Service('/gripper_control', SetBool, handle_tool_control)
    rospy.loginfo("Ready to control gripper.")
    rospy.spin()

if __name__ == "__main__":
    tool_control_server()

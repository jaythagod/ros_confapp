#!/usr/bin/env python3

import rospy
from ros_confapp.srv import processConfigAction, processConfigActionRequest, processConfigActionResponse

class coreFeatureNode():
    def __init__(self):
        pass

    def getExecutedCommand(self, req):
        rospy.loginfo('DSL Command Executed: {}'.format(req.cmd))
        return "Message Acknowledged"

def main():
    rospy.init_node("core_feature_server")
    rospy.loginfo("Listening to DSL")
    fcore = coreFeatureNode()
    getCmd = rospy.Service('get_cmd', processConfigAction, fcore.getExecutedCommand)
    print(getCmd)
    rospy.spin()

if __name__ == '__main__':
    main()


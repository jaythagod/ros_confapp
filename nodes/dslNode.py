#!/usr/bin/env python3
import rospy
from ros_confapp.srv import processConfigAction, processConfigActionRequest, processConfigActionResponse
from ros_confapp.dsl.loader import Loader


def main():
    rospy.init_node("dsl_command_exec")
    rospy.wait_for_service('get_cmd')
    ldr = Loader()
    launchRes = ldr.launch()
    #print(launchRes)

    try:
        core_config = rospy.ServiceProxy('get_cmd', processConfigAction)
        respl = core_config(launchRes)
        return respl.feedback
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed")
        rospy.loginfo(e)


if __name__ == '__main__':
    main()
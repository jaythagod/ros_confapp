import rospy
from std_msgs.msg import String

nodeName = 'parallel_gripper'
topicName = "parallel_gripper"

def pgrip_callback(data):
    rospy.loginfo("%s command published",data.data)
    if data.data == "unload":
        unloadThisFeature()
    elif data.data == "ping":
        runNode()

def unloadThisFeature():
    reason = "Parallel gripper feature unloaded from configuration"
    rospy.loginfo(reason)
    rospy.signal_shutdown(reason)
    
def runNode():
    rospy.loginfo("Parallel gripper feature executed")

def main():
    rospy.init_node(nodeName, disable_signals=True, anonymous=True)
    rospy.loginfo("Listening@para-gripper.node")
    rospy.Subscriber(topicName, String, pgrip_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
import rospy
from std_msgs.msg import String

nodeName = 'pathplanning'
topicName = "pathplanning"

def pp_callback(data):
    rospy.loginfo("%s command published",data.data)
    if data.data == "unload":
        unloadThisFeature()
    elif data.data == "ping":
        runNode()

def unloadThisFeature():
    reason = "Path planning feature unloaded from configuration"
    rospy.loginfo(reason)
    rospy.signal_shutdown(reason)
    
def runNode():
    rospy.loginfo("Path planning feature executed")

def main():
    rospy.init_node(nodeName, disable_signals=True, anonymous=True)
    rospy.loginfo("Listening@pathplanning.node")
    rospy.Subscriber(topicName, String, pp_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
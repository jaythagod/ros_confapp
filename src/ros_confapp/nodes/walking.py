import rospy
from std_msgs.msg import String

nodeName = 'walking'
topicName = "walking"

def walk_callback(data):
    rospy.loginfo("%s command published",data.data)
    if data.data == "unload":
        unloadThisFeature()
    elif data.data == "ping":
        runNode()

def unloadThisFeature():
    reason = "Walking feature unloaded from configuration"
    rospy.loginfo(reason)
    rospy.signal_shutdown(reason)
    
def runNode():
    rospy.loginfo("Walking feature executed")

def main():
    rospy.init_node(nodeName, disable_signals=True, anonymous=True)
    rospy.loginfo("Listening@walking.node")
    rospy.Subscriber(topicName, String, walk_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
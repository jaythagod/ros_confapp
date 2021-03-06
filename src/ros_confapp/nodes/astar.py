import rospy
from std_msgs.msg import String

nodeName = 'astar'
topicName = "astar"

def astar_callback(data):
    rospy.loginfo("%s command published",data.data)
    if data.data == "unload":
        unloadThisFeature()
    elif data.data == "ping":
        runNode()

def unloadThisFeature():
    reason = "Astar feature unloaded from configuration"
    rospy.loginfo(reason)
    rospy.signal_shutdown(reason)
    
def runNode():
    rospy.loginfo("Astar feature executed")

def main():
    rospy.init_node(nodeName, disable_signals=True, anonymous=True)
    rospy.loginfo("Listening@astar.node")
    rospy.Subscriber(topicName, String, astar_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
import rospy
from std_msgs.msg import String

nodeName = 'cloudaccess'
topicName = "cloudaccess"

def ca_callback(data):
    rospy.loginfo("%s command published",data.data)
    if data.data == "unload":
        unloadThisFeature()
    elif data.data == "ping":
        runNode()

def unloadThisFeature():
    reason = "Cloud access unloaded from configuration"
    rospy.loginfo(reason)
    rospy.signal_shutdown(reason)
    
def runNode():
    rospy.loginfo("Cloud access feature executed")

def main():
    rospy.init_node(nodeName, disable_signals=True, anonymous=True)
    rospy.loginfo("Listening@cloudaccess.node")
    rospy.Subscriber(topicName, String, ca_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
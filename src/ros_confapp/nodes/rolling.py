import rospy
from std_msgs.msg import String

nodeName = 'rolling'
topicName = "rolling"

def roll_callback(data):
    rospy.loginfo("%s command published",data.data)
    if data.data == "unload":
        unloadThisFeature()
    elif data.data == "ping":
        runNode()

def unloadThisFeature():
    reason = "Rolling feature unloaded from configuration"
    rospy.loginfo(reason)
    rospy.signal_shutdown(reason)
    
def runNode():
    rospy.loginfo("Rolling feature executed")

def main():
    rospy.init_node(nodeName, disable_signals=True, anonymous=True)
    rospy.loginfo("Listening@rolling.node")
    rospy.Subscriber(topicName, String, roll_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
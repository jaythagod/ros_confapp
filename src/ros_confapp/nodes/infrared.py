import rospy
from std_msgs.msg import String

nodeName = 'infrared'
topicName = "infrared"

def ir_callback(data):
    rospy.loginfo("%s command published",data.data)
    if data.data == "unload":
        unloadThisFeature()
    elif data.data == "ping":
        runNode()

def unloadThisFeature():
    reason = "Infrared feature unloaded from configuration"
    rospy.loginfo(reason)
    rospy.signal_shutdown(reason)
    
def runNode():
    rospy.loginfo("Infrared feature executed")

def main():
    rospy.init_node(nodeName, disable_signals=True, anonymous=True)
    rospy.loginfo("Listening@infrared.node")
    rospy.Subscriber(topicName, String, ir_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
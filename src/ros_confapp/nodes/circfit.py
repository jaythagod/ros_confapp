import rospy
from std_msgs.msg import String

nodeName = 'circfit'
topicName = "circfit"

def cf_callback(data):
    rospy.loginfo("%s command published",data.data)
    if data.data == "unload":
        unloadThisFeature()
    elif data.data == "ping":
        runNode()

def unloadThisFeature():
    reason = "Circular fit unloaded from configuration"
    rospy.loginfo(reason)
    rospy.signal_shutdown(reason)
    
def runNode():
    rospy.loginfo("Circular fit feature executed")

def main():
    rospy.init_node(nodeName, disable_signals=True, anonymous=True)
    rospy.loginfo("Listening@circfit.node")
    rospy.Subscriber(topicName, String, cf_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
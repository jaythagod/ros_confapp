import rospy
from std_msgs.msg import String

nodeName = 'uvc_emission'
topicName = "uvc_emission"

def ssd_callback(data):
    rospy.loginfo("%s command published",data.data)
    if data.data == "unload":
        unloadThisFeature()
    elif data.data == "ping":
        runNode()

def unloadThisFeature():
    reason = "UVC Emission feature unloaded from configuration"
    rospy.loginfo(reason)
    rospy.signal_shutdown(reason)
    
def runNode():
    rospy.loginfo("UVC Emission feature executed")

def main():
    rospy.init_node(nodeName, disable_signals=True, anonymous=True)
    rospy.loginfo("Listening@uvcemission.node")
    rospy.Subscriber(topicName, String, ssd_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
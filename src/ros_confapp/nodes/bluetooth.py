import rospy
from std_msgs.msg import String

nodeName = 'bluetooth'
topicName = "bluetooth"

def bt_callback(data):
    rospy.loginfo("%s command published",data.data)
    if data.data == "unload":
        unloadThisFeature()
    elif data.data == "ping":
        runNode()

def unloadThisFeature():
    reason = "Bluetooth unloaded from configuration"
    rospy.loginfo(reason)
    rospy.signal_shutdown(reason)
    
def runNode():
    rospy.loginfo("Bluetooth feature executed")

def main():
    rospy.init_node(nodeName, disable_signals=True, anonymous=True)
    rospy.loginfo("Listening@bluetooth.node")
    rospy.Subscriber(topicName, String, bt_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
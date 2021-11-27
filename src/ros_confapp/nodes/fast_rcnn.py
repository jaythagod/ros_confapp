import rospy
from std_msgs.msg import String

nodeName = 'fast_rcnn'
topicName = "fast_rcnn"

def fcnn_callback(data):
    rospy.loginfo("%s command published",data.data)
    if data.data == "unload":
        unloadThisFeature()
    elif data.data == "ping":
        runNode()

def unloadThisFeature():
    reason = "Fast R-CNN feature unloaded from configuration"
    rospy.loginfo(reason)
    rospy.signal_shutdown(reason)
    
def runNode():
    rospy.loginfo("Fast R-CNN feature executed")

def main():
    rospy.init_node(nodeName, disable_signals=True, anonymous=True)
    rospy.loginfo("Listening@fcnn.node")
    rospy.Subscriber(topicName, String, fcnn_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
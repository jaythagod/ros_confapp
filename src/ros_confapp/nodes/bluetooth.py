import rospy
#from ros_confapp.srv import dijkstra, dijkstraRequest, dijkstraResponse
from std_msgs.msg import String
#from ros_confapp.featx.extracts.dijkstra.runDijkstra import RunDijkstra


def callback(data):
    rospy.loginfo("I heard %s",data.data)

def main():
    
    rospy.init_node('bluetooth_feature', anonymous=True)
    rospy.loginfo("Listening@bluetooth.node")
    rospy.Subscriber("bluetooth", String, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
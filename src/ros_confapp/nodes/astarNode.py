import rospy
#from ros_confapp.srv import dijkstra, dijkstraRequest, dijkstraResponse
from std_msgs.msg import String
from ros_confapp.featx.extracts.dijkstra.runDijkstra import RunDijkstra


def astar_callback(data):
    rospy.loginfo("I heard %s",data.data)

def main():
    
    rospy.init_node('astar_feature', anonymous=True)
    rospy.loginfo("Listening@astar.node")
    rospy.Subscriber("astar", String, astar_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
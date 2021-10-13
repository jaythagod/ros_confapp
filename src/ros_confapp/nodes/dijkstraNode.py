import rospy
from std_msgs.msg import String
from ros_confapp.nodes.core import Core
from ros_confapp.code.dijkstra.runDijkstra import RunDijkstra

configCheck = Core()

def stopNode():
    pass

def dijkstra_callback(data):
    rospy.loginfo("I heard %s",data.data)
    cmdArr = data.data.split()
    checkVal = configCheck.validateTimeModeCombo(data.data)
    
    if checkVal:
        if cmdArr[0].lower() == "load":
            dijk = RunDijkstra(int(cmdArr[2]), int(cmdArr[3]), int(cmdArr[4]), int(cmdArr[5]))
            path = dijk.run()
            rospy.loginfo(path)
        elif cmdArr[0].lower() == "unload":
            pass
            #TODO: work on unload implementation
    else:
        rospy.loginfo("Feature cannot be loaded based on time and mode combination")

def main():
    
    rospy.init_node('dijkstra_feature', anonymous=True)
    rospy.loginfo("Listening@dijkstra.node")
    rospy.Subscriber("dijkstra", String, dijkstra_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
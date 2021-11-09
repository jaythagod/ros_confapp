import rospy
from std_msgs.msg import String


def callback(data):
    rospy.loginfo("I heard %s",data.data)

def main():
    
    rospy.init_node('cloudaccess_feature', anonymous=True)
    rospy.loginfo("Listening@ca.node")
    rospy.Subscriber("cloudaccess", String, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
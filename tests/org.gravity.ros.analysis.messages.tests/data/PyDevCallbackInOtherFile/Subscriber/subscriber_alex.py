import rospy
from std_msgs.msg import String
from Subscriber.functions import callback_alex

def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback_alex)

    rospy.spin()

if __name__ == '__main__':
    listener()
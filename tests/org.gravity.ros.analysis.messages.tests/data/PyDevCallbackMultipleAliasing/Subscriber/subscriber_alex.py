import rospy
from std_msgs.msg import String
from Subscriber.functions import callback_alex as callback_function

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)

    rospy.spin()


callback = callback_function

if __name__ == '__main__':
    listener()
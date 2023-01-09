import rospy
from std_msgs.msg import String
from Subscriber.functions import Callbacks as Call_class

def listener():
    
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback_function.callback_alex)

    rospy.spin()
    
callback_function = Call_class

if __name__ == '__main__':
    listener()
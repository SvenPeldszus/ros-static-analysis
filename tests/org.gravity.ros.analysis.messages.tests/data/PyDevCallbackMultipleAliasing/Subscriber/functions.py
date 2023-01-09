import rospy

def callback_alex(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
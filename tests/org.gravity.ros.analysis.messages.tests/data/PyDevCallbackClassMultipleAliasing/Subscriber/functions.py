import rospy

class Callbacks:
    @staticmethod
    def callback_alex(data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
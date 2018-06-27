#!/usr/bin/env python
import rospy
from std_msgs.msg  import String

def callback(data):
    """TODO: Docstring for callback.

    :data: TODO
    :returns: TODO

    """
    rospy.loginfo(rospy.get_caller_id() + "I heard ---{}---".format(data.data))

def listener():
    """The listner node that write the string on screen.
    :returns: TODO

    """
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', String, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()

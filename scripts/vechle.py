# This is a class that stores the internal informaiton in the car
# Written an maintained by Magnus Sorensen
# 20180802

import rospy

class vechle(object):

    """Docstring for vechle(object. """

    def __init__(self):
        """TODO: to be defined1. """
        # From rio
        self.speed_is = list()
        self.steering_is = list()
        self.sent_time_to_rio = rospy.Time()
        self.delta_tx2_rio = -1
        # to rio
        self.set_speed = 0
        self.set_steering = 0
        self.enable = 0
        self.recived_time_from_rio = rospy.Time()
        # from gui
        self.gui_connection_requierd = 1
        self.enable = 0
        self.run = 0
        self.max_speed = 3.0
        # to gui
        self.distance_front = list()
        self.distance_back = list()
        # error info
        self.error = 0
        self.error_message = str()
        # Genarl info
        self.updates_from_rio = 0

    def set_delta_tx2_rio(self, recived_time_from_rio):
        self.updates_from_rio += 1
        self.recived_time_from_rio = recived_time_from_rio
        self.delta_tx2_rio = self.sent_time_to_rio.now() - recived_time_from_rio

    def get_time(self):
        self.sent_time_to_rio = rospy.Time.now()
        return self.sent_time_to_rio


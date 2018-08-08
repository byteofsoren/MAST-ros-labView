# This is a class that stores the internal informaiton in the car
# Written an maintained by Magnus Sorensen
# 20180802

import rospy
import numpy as np

class vechle(object):

    """Docstring for vechle(object. """

    def __init__(self):
        """TODO: to be defined1. """
        # From rio
        self.speed_is = [0 for i in range(4)]
        self.steering_is = [0 for i in range(4)]
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
        self.max_speed
        self.gui_last_time = rospy.Time.now()
        self.gui_got_time = rospy.Time.now()
        # to gui
        self.distance_front = [-1 for i in range(3)]
        self.distance_back = [-1 for i in range(3)]
        # error info
        self.error = 0
        self.error_message = str()
        # Genarl info
        self.updates_from_rio = 0
        self.canny_error=0

        # PID related constants and variables
        self._prevE = 0
        self._integral = 0

        self._steeringP = 0
        self._steeringI = 0
        self._steeringD = 0

        self._speedP = 0
        self._speedI = 0
        self._speedD = 0

        self._lastTime = rospy.Time.now()

    def set_delta_tx2_rio(self, recived_time_from_rio):
        self.updates_from_rio += 1
        self.recived_time_from_rio = recived_time_from_rio
        self.delta_tx2_rio = self.sent_time_to_rio.now() - recived_time_from_rio

    def get_time(self):
        self.sent_time_to_rio = rospy.Time.now()
        return self.sent_time_to_rio

    def controller(self, currE):
        currTime = rospy.Time.now()
        ds = currTime.secs - self._lastTime.secs
        dn = currTime.nsecs - self._lastTime.nsecs
        dn = dn/1000000000 # convert to seconds
        dt = ds+dn #dt in seconds
        self._lastTime = currTime

        self._integral += currE*dt
        derivative = (currE-prevE)/dt
        prevE = currE

        # Steering PID
        desiredSteeringAngle = (self._steeringP*currE +
                                self._steeringI*self._integra +
                                self._steeringD*derivative)
        if desiredSteeringAngle > 20:
            desiredSteeringAngle = 20
        elif desiredSteeringAngle < -20:
            desiredSteeringAngle = -20

        # Speed PID
        desiredSpeed = (self.max_speed - np.absolute(self._speedP*currE +
                                                     self._speedI*self._integral +
                                                     self._speedD*derivative))
        if desiredSpeed < 0.5:
            desiredSpeed = 0.5

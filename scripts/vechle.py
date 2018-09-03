# This is a class that stores the internal informaiton in the car
# Written an maintained by Magnus Sorensen
# 20180802

import rospy
import numpy as np

class vechle(object):

    """Docstring for vechle(object. """

    def __init__(self):
        """TODO: to be defined1. """
        # From RoboRIO
        self.speed_is = [0 for i in range(4)]
        self.steering_is = [0 for i in range(4)]
        self.sent_time_to_rio = rospy.Time()
        self.delta_tx2_rio = -1
        # to RoboRIO
        self.set_speed = 0
        self.set_steering = 0
        self.enable = 0
        self.recived_time_from_rio = rospy.Time()
        # from GUI
        self.gui_connection_requierd = 1
        self.enable = 0
        self.run = 0
        self.max_speed = 3
        self.min_speed = 0.5
        self.gui_last_time = rospy.Time.now()
        self.gui_got_time = rospy.Time.now()
        # to GUI
        self.distance_front = [-1 for i in range(3)]
        self.distance_back = [-1 for i in range(3)]
        self.distance_error = 0
        # the threshold for disabling the car based on GUI connection lost in nanoseconds
        self.gui_time_threshold  = rospy.Time(secs=1.3).nsecs
        # error info
        self.error = 0
        self.error_message = str()
        # Genarl info
        self.updates_from_rio = 0
        self.canny_error=0
        self.canny_time_stamp = rospy.Time.now()
        self.canny_threshold = rospy.Time(secs=0.9).nsecs

        # PID related constants and variables
        self._maxE = 40
        self._prevE = 0
        self._integral = 0

        self._steeringP = 0.5
        self._steeringI = 0
        self._steeringD = 0

        self._speedP = 0.0625
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

    def controller(self):
        currTime = rospy.Time.now()
        ds =float(currTime.secs - self._lastTime.secs)
       # print("ds = {}, currTime {} - lastTime {}".format(ds, currTime.secs, self._lastTime.secs))
        dn = float(currTime.nsecs - self._lastTime.nsecs)
       # print("dn = {}, currTime {} - lastTime {}".format(dn, currTime.nsecs, self._lastTime.nsecs))
        dn = dn/1000000000 # convert to seconds
       # print("dn converted to = {}".format(dn))
        dt = ds+dn #dt in seconds
       # print("dt = {}, ds = {}, dn = {}".format(dt, ds, dn))
        self._lastTime = currTime

       # meanDistance = np.array(self.distance_front).mean()
       # print("meanDistance = {}".format(meanDistance))
       # if meanDistance > 600:

        self._integral += self.canny_error*dt
        derivative = (self.canny_error-self._prevE)/dt
        prevE = self.canny_error

        # Steering PID
        desiredSteeringAngle = (self._steeringP*self.canny_error +
                                self._steeringI*self._integral +
                                self._steeringD*derivative)
        if desiredSteeringAngle > 20:
            desiredSteeringAngle = 20
        elif desiredSteeringAngle < -20:
            desiredSteeringAngle = -20

        # Speed PID
        self._speedP = (self.max_speed-self.min_speed)/self._maxE
        desiredSpeed = (self.max_speed - np.absolute(self._speedP*self.canny_error +
                                                     self._speedI*self._integral +
                                                     self._speedD*derivative))
        if desiredSpeed < 0.5:
            desiredSpeed = 0.5


        # else:
        #     if self.distance_front[0] < self.distance_front[2]:
        #         desiredSteeringAngle = -20
        #     else:
        #         desiredSteeringAngle = 20
        #     desiredSpeed = -0.5

        self.set_speed = desiredSpeed
        self.set_steering = desiredSteeringAngle

    def goingSlowForward(self):
        self.set_speed = 0.5

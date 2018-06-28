#!/usr/bin/env python
# -----------------------------------------
# This is part of MAST.
# Created and mantained by Magnus Sörensen.
# -----------------------------------------
# ROS - LabView comunication and probably AI part.
# It reads stearing and speed from RoboRIO by reading.
# ros_labview_dummy:
#   from_rio:
#       float64 spead_is
#       float64 stearing_is
# It sends stearing and speed to RoboRIO by publeshing.
# ros_labviw:
#   to_rio:
#       float64 set_spead
#       float64 set_stearing
# -----------------------------------------
import rospy
from std_msgs.msg  import String
from ros_labview_dummy import from_rio
from ros_labview import to_rio

# Globals.
message_to_rio = to_rio()
message_from_rio = from_rio()
pub_handle = rospy.Publisher('control_to_rio', data_class=to_rio)
update_rate = rospy.Rate(100) #100hz
rospy.init_node('form_rio', anonymous=True)


def send_to_rio(steering=0,speed=0):
    """TODO: for send_to_rio.
    :returns: none

    """
    message_to_rio.set_speed=speed
    message_to_rio.set_steering=steering
    message_to_rio.time_frame=rospy.get_time()
    rospy.loginfo(message_to_rio)
    pub_handle.publish(message_to_rio)

def read_from_rio():
    """Reding from rio is done heare
    :returns: [steering,speed,message_str]

    """
    pass

def main():
    '''This is the main loop
    '''
    while not rospy.is_shutdown():
        # Update message to Roborio
        # ---- Write control code here ----
        # Write to RoboRIO
        send_to_rio(1.0,3.0)
        # Read from RoboRIO
        update_rate.sleep()
    pass

if __name__ == "__main__":
    main()

#!/usr/bin/env python
# -----------------------------------------
# This is part of MAST.
# Created and mantained by Magnus Sorensen.
# -----------------------------------------
# ROS - LabView comunication and probably AI part.
# It reads stearing and speed from RoboRIO by reading.
# ros_labview_dummy:
#   from_rio:
#       float64 spead_is
#       float64 stearing_is
# It sends stearing and speed to RoboRIO by publishing.
# ros_labviw:
#   to_rio:
#       float64 set_spead
#       float64 set_stearing
# -----------------------------------------
import rospy
from std_msgs.msg  import String
from ros_labview_dummy.msg import from_rio
from ros_labview.msg import to_rio

# Settings:
publish_from_tx2_to_rio_name = 'control_to_rio'
subscibe_from_rio_to_tx2_name = 'read_rio'

# Globals.
message_to_rio = to_rio()
message_from_rio = from_rio()
# Create a publisher node so the rio can subsribe on the destinations.
pub_handle = rospy.Publisher(publish_from_tx2_to_rio_name, data_class=to_rio, queue_size=1)
rospy.init_node(subscibe_from_rio_to_tx2_name, anonymous=True)
update_rate = rospy.Rate(100) #100hz
# Subscribe to the data from rio
speed_is=0
steering_is=0

def send_to_rio(steering=0,speed=0):
    """TODO: for send_to_rio.
    :returns: none

    """
    message_to_rio.set_speed = speed
    message_to_rio.set_steering = steering
    message_to_rio.time_frame = rospy.get_time()
    print("Sending speed={}, steering={}".format(speed,steering))
    pub_handle.publish(message_to_rio)

def read_from_rio(data):
    """Reding from rio is done heare
    :returns: [steering,speed]

    """
    speed_is=data.speed_is
    steering_is=data.steering_is
    print('data.speed_is = {}, data.steering_is = {}' .format(data.speed_is, data.steering_is))
    #pass

def main():
    '''This is the main loop
    '''
    rospy.Subscriber(subscibe_from_rio_to_tx2_name,from_rio, read_from_rio )
    counter = 0.0
    while not rospy.is_shutdown():
        # Update message to Roborio
        # ---- Write control code here ----
        # Write to RoboRIO
        send_to_rio(1.0,counter)
        # Read from RoboRIO
        #print('roborio speed is ={}, stearing is = {}'.format(speed_is, steering_is))
        update_rate.sleep()
        #rospy.rostime.wallsleep(0.1)
        counter += 1
    #pass

if __name__ == "__main__":
    main()

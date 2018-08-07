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
from vechle import vechle
from std_msgs.msg  import String
from ros_labview_dummy.msg import from_rio
from ros_labview.msg import to_rio
from ros_labview.msg import vehicle_settings
from cannylive.msg import errorDistances

# topics:
publish_from_tx2_to_rio_name = 'control_to_rio'
subscibe_from_rio_to_tx2_name = 'read_rio'
subscribe_vehicle_settings = 'vehicle_settings'
subscribe_canny_name = 'cannylive'

# Globals.
message_to_rio = to_rio()
message_from_rio = from_rio()
message_settings = vehicle_settings()
message_canny = errorDistances()

# Create a publisher node so the rio can subsribe on the destinations.
tx2_to_rio_pub_handle = rospy.Publisher(publish_from_tx2_to_rio_name, data_class=to_rio, queue_size=1)
#tx2_to_ext_pub_handle = rospy.Publisher(publish_vehicle_status, data_class=vehicle_status, queue_size=1)

# Create the nodes in the car.
rospy.init_node('tx2_mani.py', anonymous=True)
update_rate = rospy.Rate(100) #100hz
# Subscribe to the data from rio

def send_to_rio():
    """This function sends the car information to the RoboRIO
    :returns: none

    """
    global car
    message_to_rio.set_speed = car.set_speed
    message_to_rio.set_steering = car.set_steering
    message_to_rio.time_frame = car.get_time()
    # Is the car in enable mode?
    if not car.enable == 0:
        message_to_rio.enable = 1
        pass
    else:
        message_to_rio.enable = 0
    if car.updates_from_rio % 60 == 0:
        rospy.loginfo("Sent to RoboRIO enable = {enable}".format(enable=message_to_rio.enable))
    # Publish the message to roboRIO
    tx2_to_rio_pub_handle.publish(message_to_rio)

def read_from_rio(data):
    """Reeding from RoboRIO is done here
    :returns: none

    """
    global car
    if car.updates_from_rio % 30 == 0:
        rospy.loginfo("Read from rio")
    car.speed_is = data.speed_is
    car.steering_is = data.steering_is
    car.set_delta_tx2_rio(data.delta)
    car.error_message += " " + str(data.error_message)
    car.error += data.error

def read_canny(canny):
    """ Reads error from canny_live module
    """
    # This don't work yet
    car.canny_error = canny.errorDist
    rospy.loginfo("canny error = {}".format(car.canny_error))



def read_settings(settings):
    """Reads settings from GUI
    :returns: none

    """
    # The global car class is used to store the data.
    global car

    car.enable = settings.enable
    car.run = settings.run
    if not car.enable == 0:
        rospy.logwarn("-------Car is enabled---------")
    elif car.updates_from_rio % 20 == 0:
        rospy.loginfo("read_settings from GUI")


def main():
    '''This is the main loop
    '''
    # Start subscribing to RoboRIO and Vechile settings.
    rospy.Subscriber(subscibe_from_rio_to_tx2_name,from_rio, read_from_rio )
    rospy.Subscriber(subscribe_vehicle_settings,vehicle_settings, read_settings)
    # Subscribe to canny live error data <Disabled right now>
    rospy.Subscriber(subscribe_canny_name,errorDistances, read_canny)
    counter = 0.0
    global car
    while not rospy.is_shutdown():
        if counter % 60 == 0:
            rospy.loginfo("-- Spin in main counter={c}--".format(c=counter))
        # Send data to RoboRIO
        if counter % 300  == 0:
            rospy.loginfo("controller run")
            #car.controller()
        send_to_rio()
        # sleep
        update_rate.sleep()
        counter += 1
    #pass

car = vechle()
if __name__ == "__main__":
    main()

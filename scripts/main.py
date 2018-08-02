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
from ros_labview.msg import vehicle_status
from ros_labview.msg import vehicle_settings

# topics:
publish_from_tx2_to_rio_name = 'control_to_rio'
subscibe_from_rio_to_tx2_name = 'read_rio'
publish_vehicle_status = 'vehicle_status'
subscribe_vehicle_settings = 'vehicle_settings'

# Globals.
message_to_rio = to_rio()
message_from_rio = from_rio()
message_status = vehicle_status()
message_settings = vehicle_settings()
# Create a publisher node so the rio can subsribe on the destinations.
tx2_to_rio_pub_handle = rospy.Publisher(publish_from_tx2_to_rio_name, data_class=to_rio, queue_size=1)
tx2_to_ext_pub_handle = rospy.Publisher(publish_from_tx2_to_rio_name, data_class=vehicle_status, queue_size=1)

# Create the nodes in the car.
rospy.init_node(subscibe_from_rio_to_tx2_name, anonymous=True)
update_rate = rospy.Rate(100) #100hz
# Subscribe to the data from rio

def send_to_rio():
    """TODO: for send_to_rio.
    :returns: none

    """
    global car
    message_to_rio.set_speed = car.set_speed
    message_to_rio.set_steering = car.set_steering
    message_to_rio.time_frame = car.get_time()
    message_to_rio.enable = car.enable
    if not car.enable == 0:
        rospy.logwarn("--Sent enable to roborio --")
    tx2_to_rio_pub_handle.publish(message_to_rio)
    rospy.loginfo("Sent to rio")

def read_from_rio(data):
    """Reding from rio is done heare
    :returns: [steering,speed]

    """
    global car
    car.speed_is = data.speed_is
    car.steering_is = data.steering_is
    car.set_delta_tx2_rio(data.delta)
    car.error_message += " " + str(data.error_message)
    car.error += data.error
    rospy.loginfo("Read from rio")
    #speed_is=data.speed_is
    #steering_is=data.steering_is
    #print('data.speed_is = {}, data.steering_is = {}' .format(data.speed_is, data.steering_is))
    #pass


def send_status():
    """This publish the status of the car on the topic vehicle_status
    :returns: None

    """
    global car
    message_status.speed_is = car.speed_is
    message_status.steering_is = car.steering_is
    message_status.distance_front = car.distance_back
    message_status.distance_back = car.distance_back
    message_status.error_id = car.error
    message_status.error_message = car.error_message
    message_status.time_frame = rospy.Time.now()
    message_status.delta_tx2_rio = car.get_time()
    tx2_to_ext_pub_handle.publish(message_status)
    rospy.loginfo("Sent status to topic {topic}".format(topic=publish_vehicle_status))

def read_settings(settings):
    """Reads settings from guis
    :returns: TODO

    """
    global car

    car.enable = settings.enable
    if not car.enable == 0:
        rospy.logwarn("-------Car is enabeld---------")
    elif car.updates_from_rio % 20 == 0:
        rospy.loginfo("read_settings from GUI")


def main():
    '''This is the main loop
    '''
    rospy.Subscriber(subscibe_from_rio_to_tx2_name,from_rio, read_from_rio )
    rospy.Subscriber(subscribe_vehicle_settings,vehicle_settings, read_settings)
    counter = 0.0
    global car
    while not rospy.is_shutdown():
        # Update message to Roborio
        # ---- Write control code here ----
        # Write to RoboRIO
        car.set_speed = counter
        car.set_steering = 12
        send_to_rio()
        send_status()
        # Read from RoboRIO
        #print('roborio speed is ={}, stearing is = {}'.format(speed_is, steering_is))
        update_rate.sleep()
        #rospy.rostime.wallsleep(0.1)
        counter += 1
    #pass

car = vechle()
if __name__ == "__main__":
    main()

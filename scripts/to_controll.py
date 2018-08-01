#!/usr/bin/env python
# -----------------------------------------
# This is part of MAST.
# Created and mantained by Magnus Sorensen.
# -----------------------------------------
# ROS node to send data from the system to a HUD.
# It reads stearing and speed from RoboRIO by creating
# a publisher that publish the vehicle_status.msg:
#   float64 speed_is
#   float64[4] steering_is [FL,FR,BL,BR]
#   float64[3] distance_front [L,M,R]
#   float64[3] distance_back [L,M,R]
#   int8 error_id
#   string error_message
# -----------------------------------------
import numpy as np
import rospy
from std_msgs.msg  import String
from ros_labview_dummy.msg import from_rio
#from ros_labview.msg import to_rio
from ros_labview.msg import vehicle_status
from distance_sensor.msg import sensor_values


# Settings:
#publish_from_tx2_to_rio_name = 'control_to_rio'
publish_status_name = 'vehicle_status'
subscibe_from_rio_to_tx2_name = 'read_rio'
subscrib_to_distance_name = 'sensor_distance'


# Globals.
#message_to_rio = to_rio()
message_from_rio = from_rio()
message_status = vehicle_status()
front_sensors = sensor_values()
# Create a publisher node so the rio can subsribe on the destinations.
#pub_handle = rospy.Publisher(publish_from_tx2_to_rio_name, data_class=to_rio, queue_size=1)
pub_handle = rospy.Publisher(publish_status_name, data_class=vehicle_status, queue_size=1)
rospy.init_node(subscibe_from_rio_to_tx2_name, anonymous=True)
update_rate = rospy.Rate(10) #100hz
# Subscribe to the data from rio
status = 0       # This is a vehicle_status struct
front_dist = 0   # This is a front_sensors struct

def send_status():
    """TODO: for send status to a eventual controll gui.
    :returns: none

    """
    # The first line make shures that at least one iteration of the data have bean done
    # and there for there exist data on the nodes.
    rospy.loginfo("Send_status")
    if (type(status) != type(0)) and (type(front_dist) != type(0)):
        rospy.loginfo("Status ok -> sending")
        message_status.speed_is = status.speed_is
        message_status.steering_is = [status.steering_is, status.steering_is, 0 , 0]
        message_status.distance_frant = [front_dist.left_sensor, front_dist.middle_sensor, front_dist.right_sensor]
        message_status.time_frame = rospy.get_time()
        sensor_error =  np.sum(front_dist.error)
        sensor_error_message = ""
        if not sensor_error == 0:
            if not front_dist.error[0] == 0:
                sensor_error_message += "Left Sensor error "
                rospy.logwarn("Left Sensor error")
            if not front_dist.error[1] == 0:
                sensor_error_message += "Middle Sensor error "
                rospy.logwarn("Middle Sensor error")
            if not front_dist.error[2] == 0:
                sensor_error_message += "Right Sensor error "
                rospy.logwarn("Right Sensor error")
            pass

        message_status.error_id = status.error_id + sensor_error
        message_status.error_message += sensor_error_message

        pub_handle.publish(message_status)
        rospy.loginfo("Sending speed={speed}, stearing={stearing} to gui".format(speed=status.speed_is, stearing=status.stearing_is))

def read_from_rio(data):
    """Reding from rio is done heare
    :returns: [steering,speed]

    """
    status = data
    speed_is=data.speed_is
    steering_is=data.steering_is
    print('data.speed_is = {}, data.steering_is = {}' .format(data.speed_is, data.steering_is))
    #pass

def read_sensors(sens):
    """Reads the lasersensor values from the sensor_values node.
    :returns:nothing

    """
    front_dist = sens
    pass

def main():
    '''This is the main loop
    '''
    counter = 0.0
    while not rospy.is_shutdown():
        # Update message to Roborio
        # ---- Write control code here ----
        # Write to RoboRIO
        # send_to_rio(1.0,counter)
        send_status()
        # Read from RoboRIO
        #print('roborio speed is ={}, stearing is = {}'.format(speed_is, steering_is))
        update_rate.sleep()
        #rospy.rostime.wallsleep(0.1)
        counter += 1
    #pass

if __name__ == "__main__":
    rospy.loginfo("Start sub from tx2")
    rospy.Subscriber(subscibe_from_rio_to_tx2_name,from_rio, read_from_rio )
    rospy.loginfo("Start sub from distance sensors")
    rospy.Subscriber(subscrib_to_distance_name, sensor_values, read_sensors )
    main()

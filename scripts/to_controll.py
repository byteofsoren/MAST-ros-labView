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
from vechle import vechle
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
rospy.init_node('tx2_to_controll.py', anonymous=True)
update_rate = rospy.Rate(10) #100hz
# Subscribe to the data from rio
status = 0       # This is a vehicle_status struct
front_dist = 0   # This is a front_sensors struct
counter = 0

def send_status():
    """TODO: for send status to a eventual controll gui.
    :returns: none

    """
    # The first line make shures that at least one iteration of the data have bean done
    # and there for there exist data on the nodes.
    global counter
    if counter % 30 == 0 :
        rospy.loginfo("Send_status")
    global car
    #message_status.speed_is = [1,2,3]
    message_status.speed_is = car.speed_is

    message_status.steering_is = car.steering_is
    message_status.distance_front = car.distance_front
    message_status.distance_back =[0,0,0]
    if int(car.error) > 100:
        car.error = 100
        rospy.logwarn("#Error was bigger then 200")
    message_status.error_id = int(car.error)
    message_status.error_message = car.error_message
    message_status.time_frame = rospy.Time.now()
    #rospy.loginfo("delta_tx2_rio={delta}, type(set_delta_tx2_rio)={t}".format(delta=car.delta_tx2_rio, t=type(car.delta_tx2_rio)))
    # I know how to solve the bug old.nsecs() - new.nsecs()
    ##message_status.delta_tx2_rio = car.delta_tx2_rio
    pub_handle.publish(message_status)



def read_from_rio(rio):
    """Reding from rio is done heare
    :returns: [steering,speed]

    """
    global counter
    global car
    car.speed_is = rio.speed_is
    car.steering_is = rio.steering_is
    car.error = rio.error
    car.error_message = rio.error_message
    car.set_delta_tx2_rio(rio.delta)
    if counter % 30 == 0 :
        rospy.loginfo("read_from_rio speed_is={s}, steering_is={t}".format(s=car.speed_is, t=car.steering_is))
    #pass

def read_sensors(sens):
    """Reads the lasersensor values from the sensor_values node.
    :returns:nothing

    """
    global car
    global counter
    car.distance_front = [sens.left_sensor, sens.middle_sensor, sens.right_sensor]
    t = np.sum(sens.error)
    car.error = t
    car.distance_error = t
    if counter % 30 == 0:
        rospy.loginfo("Sensor values {left},{mid},{right}".format(left=sens.left_sensor, mid=sens.middle_sensor, right=sens.right_sensor))
    sensor_error_message = ""
    if not car.error == 0:
        pass
        if not car.distance_error == 0:
            if not sens.error[0] == 0:
                sensor_error_message += "Left Sensor error "
                rospy.logwarn("Left Sensor error")
            if not sens.error[1] == 0:
                sensor_error_message += "Middle Sensor error "
                rospy.logwarn("Middle Sensor error")
            if not sens.error[2] == 0:
                sensor_error_message += "Right Sensor error "
                rospy.logwarn("Right Sensor error")
        car.error_message = sensor_error_message

def main():
    '''This is the main loop
    '''
    global counter
    while not rospy.is_shutdown():
        # Update message to Roborio
        # ---- Write control code here ----
        send_status()
        # Read from RoboRIO
        #print('roborio speed is ={}, stearing is = {}'.format(speed_is, steering_is))
        update_rate.sleep()
        #rospy.rostime.wallsleep(0.1)
        counter += 1
    #pas

car = vechle()
if __name__ == "__main__":
    rospy.loginfo("Start sub from tx2")
    rospy.Subscriber(subscibe_from_rio_to_tx2_name,from_rio, read_from_rio )
    rospy.loginfo("Start sub from distance sensors")
    rospy.Subscriber(subscrib_to_distance_name, sensor_values, read_sensors )
    main()

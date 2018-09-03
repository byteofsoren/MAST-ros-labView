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
import numpy
from vechle import vechle
from std_msgs.msg  import String
from ros_labview_dummy.msg import from_rio
from ros_labview.msg import to_rio
from ros_labview.msg import vehicle_settings
from ros_labview.msg import stop
from distance_sensor.msg import sensor_values
from cannylive.msg import errorDistances

# topics:
publish_from_tx2_to_rio_name = 'control_to_rio'
subscibe_from_rio_to_tx2_name = 'read_rio'
subscribe_vehicle_settings = 'vehicle_settings'
subscribe_canny_name = 'errorDistances'
#publish_emstop_name = 'emergency_stop_internal'
publish_emstop_name = 'emergency_stop_external'

# Globals.
message_to_rio = to_rio()
message_from_rio = from_rio()
message_settings = vehicle_settings()
message_canny = errorDistances()
message_stop = stop()
message_from_terra = sensor_values()

# Create a publisher node so the rio can subsribe on the destinations.
tx2_to_rio_pub_handle = rospy.Publisher(publish_from_tx2_to_rio_name, data_class=to_rio, queue_size=1)
tx2_emstop_pub_handle = rospy.Publisher(publish_emstop_name, data_class=stop, queue_size=1)
#tx2_to_ext_pub_handle = rospy.Publisher(publish_vehicle_status, data_class=vehicle_status, queue_size=1)

# Create the nodes in the car.
rospy.init_node('tx2_mani.py', anonymous=True)
update_rate = rospy.Rate(100) #100hz

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
    delta_tx2 = (rospy.Time.now() - car.sent_time_to_rio).nsecs
    rospy.loginfo("delta_tx2 {t}".format(t=delta_tx2))

def read_canny(canny):
    """ Reads error from canny_live module
    """
    car.canny_error = canny.errorDist
    car.canny_time_stamp = rospy.Time.now()
    rospy.loginfo("canny error = {}".format(car.canny_error))

def read_settings(settings):
    """Reads settings from GUI
    :returns: none
    """
    # The global car class is used to store the data.
    global car
    car.gui_got_time = rospy.Time.now()
    car.enable = settings.enable
    car.run = settings.run
    car.max_speed = settings.max_speed
    rospy.loginfo("Car max speed is {}, Settings max speed is {}".format(car.max_speed, settings.max_speed))
    car.gui_connection_requierd = settings.gui_connection_requierd
    rospy.loginfo("read_settings from GUI")
    if not car.enable == 0:
        rospy.logwarn("-------Car is enabled---------")
    elif car.updates_from_rio % 20 == 0:
        pass


def write_emstop_on_no_gui():
    """Publish a em stop if GUI lost connection.
    :returns: None
    """
    global car
    nowtime = rospy.Time.now()
    diff = (nowtime - car.gui_got_time).nsecs
    # if the car is enabled and then loses connection
    # then send a emergency stop
    enable = car.enable == 1
    gui = car.gui_connection_requierd == 1
    diff_true = diff > car.gui_time_threshold
    diff_true = False
    #if (car.enable == 1) and (car.gui_connection_requierd == 1) and (diff > car.gui_time_threshold):
    if (enable) and (gui) and (diff_true):
        rospy.logerr("----Lost connection with GUI ------")
        rospy.logerr("enable={e}, gui={g}, diff_true={d}, diff={dt}".format(e=enable,g=gui,d=diff_true, dt=rospy.Time(nsecs=diff).secs))
        message_stop.stop = 1
        tx2_emstop_pub_handle.publish(message_stop)

def read_distance_sensors(sensors):
    """Reading distance sensors from distsens node

    :sensors: TODO
    :returns: TODO

    """
    global car
    car.distance_front = [sensors.left_sensor, sensors.middle_sensor, sensors.right_sensor]
    rospy.loginfo("distance_front{}".format(car.distance_front))
    car.distance_error = sensors.error
    l = ['left error', 'middle error', 'right error']
    c = 0
    for e in car.distance_error:
        if not e == 0:
            rospy.logwarn(l[c])
        c += 1


def main():
    '''This is the main loop
    '''
    # Start subscribing to RoboRIO and Vechile settings.
    rospy.Subscriber(subscibe_from_rio_to_tx2_name,from_rio, read_from_rio )
    rospy.Subscriber(subscribe_vehicle_settings,vehicle_settings, read_settings)
    # Subscribe to canny live error data
    rospy.Subscriber(subscribe_canny_name,errorDistances, read_canny)
    # Subscribe to distance laser node
    rospy.Subscriber("sensor_distance",sensor_values,read_distance_sensors)
    counter = 0.0
    global car
    while not rospy.is_shutdown():
        if counter % 60 == 0:
            rospy.loginfo("-- Spin in main counter={c}--".format(c=counter))
        # Send data to RoboRIO
        car.controller()
        send_to_rio()
        write_emstop_on_no_gui()
        canny_diff = (rospy.Time.now() - car.canny_time_stamp).nsecs
        if  canny_diff > car.canny_threshold:
            rospy.logwarn(" [Canny time out] [{time}]".format(time=rospy.Time(canny_diff).secs))
        # sleep
        update_rate.sleep()
        counter += 1
    #pass

car = vechle()
if __name__ == "__main__":
    main()

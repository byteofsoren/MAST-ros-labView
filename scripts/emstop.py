#!/usr/bin/env python
# -----------------------------------------
# This is part of MAST.
# Created and mantained by Ulrik Akesson.
# -----------------------------------------
#     ROS - LabVIEW communication
# This script is used to recive and forward the emergency stop signal
# from the GUI PC via an nvidia TX2 to the roboRIO.
# Topics:
#   emergency_stop_external
#   emergency_stop_internal
# Messages:
#   vehicle_emergency_stop:
#       int8 emergency_stop = 0
# -----------------------------------------

import rospy
import rosnode
from time import sleep
from ros_labview.msg import stop

# Settings:
stopExternal = 'emergency_stop_external'
stopInternal = 'emergency_stop_internal'

# Globals:
stopMsg = stop()
emStop = 0

# Create publisher
pubHandle = rospy.Publisher(stopInternal, data_class=stop, queue_size = 1)
# Create subscriber
rospy.init_node(stopExternal, anonymous=True)
# Set refresh rate
updateRate = rospy.Rate(100) #Hz

def readEmergencyStop(data):
    """Subscribes on the emergency stop value from the GUI PC
       Updates the global variable emStop for the publisher

    """
    global emStop

    if emStop == 0:
        emStop = data.stop
        rospy.logwarn("-- readEmergencyStop --")
    else:
        rospy.logerr("--readEmergencyStop -- STOP--")
    rospy.loginfo("readEmergencyStop was {emstop}".format(emstop=(emStop)))

def writeEmergencyStop():
    """TODO: Docstring for writeEmergencyStop(.
    :returns: TODO
    """
    if emStop != 0:
        stopMsg.stop = 1
        rospy.logwarn('Stop have been triggered {}'.format(emStop))
        pubHandle.publish(stopMsg)


def main():
    rospy.Subscriber(stopExternal, stop, readEmergencyStop )
    counter = 0
    rospy.loginfo("[mast emStop init]")
    while (not rospy.is_shutdown()) and (emStop == 0):
        if not emStop == 0:
            rospy.logerr("main emstop isn't zero")
        writeEmergencyStop()
        updateRate.sleep()
        counter += 1
    rospy.logwarn("[Shut down in 5 sec]")
    writeEmergencyStop()
    sleep(5)
    nodes = rosnode.get_node_names()
    rosnode.kill_nodes(nodes)

if __name__ == "__main__":
    print("Starting emstop module")
    main()

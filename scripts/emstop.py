#!/usr/bin/env python
# -----------------------------------------
# This is part of MAST.
# Created and mantained by Ulrik Åkesson.
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
    emStop = data.stop

def writeEmergencyStop():
    """TODO: Docstring for writeEmergencyStop(.
    :returns: TODO

    """
    stopMsg.stop = emStop
    if emStop != 0:
        rospy.logwarn('Stop have been triggered {}'.format(emStop))
    pubHandle.publish(stopMsg)

def main():
    rospy.Subscriber(stopExternal, stop, readEmergencyStop )
    while not rospy.is_shutdown():
        writeEmergencyStop()
        updateRate.sleep()

if __name__ == "__main__":
    main()

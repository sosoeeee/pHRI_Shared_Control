#!/usr/bin/env python3
import sys
import os

sys.path.append(os.path.dirname(__file__)) # when using launch file, must be global path :( so directly use "os.path.dirname(__file__)"

from Actuator import Actuator
from simulateActuator import simActuator
import rospy

if __name__ == '__main__':
    try:
        rospy.init_node('actuator', anonymous=False)
        simulateFlag = rospy.get_param("/use_simulate", False)
        if simulateFlag:
            actuator = simActuator()
        else:
            actuator = Actuator()
        actuator.run()

    except rospy.ROSInterruptException:
        pass

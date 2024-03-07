#!/usr/bin/env python3
import sys
import os

# Add the current directory to the path module, in order to import the LocalPlannerClass
sys.path.append(os.path.dirname(__file__))

import rospy
from ImpedanceController import ImpedanceController

if __name__ == '__main__':
    try:
        rospy.init_node('controller', anonymous=False)
        controller_type = rospy.get_param("/controller_type", "Impedance")
        if controller_type == "Impedance":
            controller = ImpedanceController()

        controller.run()

    except rospy.ROSInterruptException:
        pass
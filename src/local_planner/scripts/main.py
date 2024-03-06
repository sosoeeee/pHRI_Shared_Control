#!/usr/bin/env python3
import sys
import os

# Add the current directory to the path module, in order to import the LocalPlannerClass
sys.path.append(os.path.dirname(__file__))

from BaseLocalPlanner import BaseLocalPlanner
import rospy


if __name__ == '__main__':
    try:
        rospy.init_node('local_planner', anonymous=False)
        localPlanner = BaseLocalPlanner()
        localPlanner.run()

    except rospy.ROSInterruptException:
        pass

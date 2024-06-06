#!/usr/bin/env python3
import sys
import os

# Add the current directory to the path module, in order to import the LocalPlannerClass
sys.path.append(os.path.dirname(__file__))

from MinimumLocalPlanner import MinimumLocalPlanner
from uniformPlanner import UniformPlanner
import rospy


if __name__ == '__main__':
    try:
        rospy.init_node('local_planner', anonymous=False)

        # localPlanner = MinimumLocalPlanner()

        localPlanner = UniformPlanner()

        localPlanner.run()

    except rospy.ROSInterruptException:
        pass

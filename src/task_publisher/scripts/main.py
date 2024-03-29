#!/usr/bin/env python3
import sys
import os

# Add the current directory to the path module
sys.path.append(os.path.dirname(__file__))

from taskServers import PubGoalActionServer
from taskServers import PubPathActionServer
from taskServers import PubTrajActionServer
import rospy


if __name__ == '__main__':
    try:
        rospy.init_node('task_servers', anonymous=False)
        pubGoalActionServer = PubGoalActionServer('ReachGoal')
        pubPathActionServer = PubPathActionServer('FollowPath')
        pubTrajActionServer = PubTrajActionServer('FollowTraj')
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

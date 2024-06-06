#!/usr/bin/env python3
# This file is the implementation of the Minimum jerk/snap trajectory planner

# when using launch file, must set sys.path.append(os.path.dirname(__file__)) to use classes in the same package
import sys
import os

sys.path.append(os.path.dirname(__file__))

from BaseLocalPlanner import BaseLocalPlanner
import numpy as np
import time

import rospy


class UniformPlanner(BaseLocalPlanner):
      def __init__(self):
            super(UniformPlanner, self).__init__()

      def initPlanner(self):
            pass

      def planTrajectory(self):
            start_point = np.array(self.start_pos)
            end_point = np.array(self.goal_pos)

            diff = end_point - start_point
            timeLen = int(self.total_time * self.control_frequency)

            trajectory = np.zeros((len(start_point) * 2, timeLen))
            
            for i in range(timeLen):
                  trajectory[0:3, i] = start_point + diff * i / timeLen
                  trajectory[3:6, i] = diff / self.total_time
            

            return trajectory

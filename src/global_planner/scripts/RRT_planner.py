#!/usr/bin/env python3
# This file is the implementation of the RRT planner

# when using launch file, must set sys.path.append(os.path.dirname(__file__)) to use classes in the same package
import sys
import os

sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.dirname(__file__) + "/rrt-algorithms")

from GlobalPlannerClass import GlobalPlanner
import numpy as np
import time

from visualization_msgs.msg import Marker
import rospy

# RRT package
from rrt_algorithms.rrt.rrt import RRT
from rrt_algorithms.search_space.search_space import SearchSpace


class RRTPlanner(GlobalPlanner):
    def __init__(self):
        self.r = None
        self.checkGoalProb = None
        self.maxIterNum = None
        self.step = None
        self.searchSpace = None
        self.normlizedObstacles = None

        super(RRTPlanner, self).__init__()

    def initPlanner(self):
        # RRT搜索步长
        self.step = rospy.get_param('/RRT_planner/step_size', 0.1)
        # RRT最大迭代步数
        self.maxIterNum = rospy.get_param('/RRT_planner/max_iter', 10000)
        # 检查是否可以连接到目标点的概率
        self.checkGoalProb = rospy.get_param('/RRT_planner/check_goal_prob', 0.01)
        # 在检查线段是否与障碍物相撞的时候，采样的点数
        self.r = rospy.get_param('/RRT_planner/line_collision_check_resolution', 1)

        # 搜索空间尺寸
        self.searchSpace = rospy.get_param('/RRT_planner/search_space', {'x': (0, 0), 'y': (0, 0), 'z': (0, 0)})
        self.searchSpace = np.array([self.searchSpace['x'], self.searchSpace['y'], self.searchSpace['z']])

    def obstaclsNormalizaion(self):
        self.normlizedObstacles = []
        for obstacle in self.obstacles.markers:
            if obstacle.type == Marker.CUBE:
                self.normlizedObstacles.append([obstacle.pose.position.x - obstacle.scale.x / 2,
                                                obstacle.pose.position.y - obstacle.scale.y / 2,
                                                obstacle.pose.position.z - obstacle.scale.z / 2,
                                                obstacle.pose.position.x + obstacle.scale.x / 2,
                                                obstacle.pose.position.y + obstacle.scale.y / 2,
                                                obstacle.pose.position.z + obstacle.scale.z / 2])

    def planPath(self):
        # RRT算法
        startTime = time.time()

        # update start and goal
        x_init = (self.start[0], self.start[1], self.start[2])
        x_goal = (self.goal[0], self.goal[1], self.goal[2])

        # update obstacles
        self.obstaclsNormalizaion()

        X = SearchSpace(self.searchSpace, self.normlizedObstacles)

        rrt = RRT(X, np.array([(self.step, 1)]), x_init, x_goal, self.maxIterNum, self.r, self.checkGoalProb)

        self.path = np.array(rrt.rrt_search())

        endTime = time.time()
        rospy.loginfo("RTT finished: " + str(endTime - startTime))

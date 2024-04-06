#!/usr/bin/env python3
# This file is the implementation of the RRT planner

# when using launch file, must set sys.path.append(os.path.dirname(__file__)) to use classes in the same package
import sys
import os

sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.dirname(__file__) + "/rrt-algorithms")

from BaseGlobalPlanner import BaseGlobalPlanner
import numpy as np
import time

from visualization_msgs.msg import Marker
import rospy

# RRT package
from rrt_algorithms.rrt.rrt import RRT
from rrt_algorithms.rrt.rrt_star_bid_h import RRTStarBidirectionalHeuristic
from rrt_algorithms.rrt.rrt_star_bid import RRTStarBidirectional
from rrt_algorithms.rrt.rrt_star import RRTStar
from rrt_algorithms.search_space.search_space import SearchSpace


class RRTPlanner(BaseGlobalPlanner):
    def __init__(self):
        self.r = None
        self.checkGoalProb = None
        self.maxIterNum = None
        self.step = None
        self.searchSpace = None
        self.normalizedObstacles = None

        super(RRTPlanner, self).__init__()

    def initPlanner(self):
        # RRT搜索步长
        self.step = rospy.get_param('/RRT_planner/step_size', 0.1)
        # RRT最大迭代步数
        self.maxIterNum = rospy.get_param('/RRT_planner/max_iter', 10000)
        # 检查是否可以连接到目标点的概率
        self.checkGoalProb = rospy.get_param('/RRT_planner/check_goal_prob', 0.01)
        # 在检查线段是否与障碍物相撞的时候，采样的点间距
        self.r = rospy.get_param('/RRT_planner/line_collision_check_resolution', 0.1)

        # 搜索空间尺寸
        self.searchSpace = rospy.get_param('/RRT_planner/search_space', {'x': [0, 0], 'y': [0, 0], 'z': [0, 0]})
        self.searchSpace = np.array([self.searchSpace['x'], self.searchSpace['y'], self.searchSpace['z']])

    def obstaclesNormalization(self):
        self.normalizedObstacles = []
        for obstacle in self.obstacles:
            if obstacle.type == Marker.CUBE or obstacle.type == Marker.SPHERE: # treat sphere as cube
                self.normalizedObstacles.append([obstacle.pose.position.x - obstacle.scale.x / 2 - self.obstacles_dilate,
                                                 obstacle.pose.position.y - obstacle.scale.y / 2 - self.obstacles_dilate,
                                                 obstacle.pose.position.z - obstacle.scale.z / 2 - self.obstacles_dilate,
                                                 obstacle.pose.position.x + obstacle.scale.x / 2 + self.obstacles_dilate,
                                                 obstacle.pose.position.y + obstacle.scale.y / 2 + self.obstacles_dilate,
                                                 obstacle.pose.position.z + obstacle.scale.z / 2 + self.obstacles_dilate])
            else:
                rospy.logerr("Obstacle type not supported: " + str(obstacle.type))

    def planPath(self):
        # RRT算法
        startTime = time.time()

        # update start and goal
        x_init = (self.start[0], self.start[1], self.start[2])
        x_goal = (self.goal[0], self.goal[1], self.goal[2])

        # update obstacles  
        self.obstaclesNormalization()

        X = SearchSpace(self.searchSpace, self.normalizedObstacles)

        # rrt = RRT(X, self.step, x_init, x_goal, self.maxIterNum, self.r, self.checkGoalProb)
        # self.path = np.array(rrt.rrt_search())

        # more smoooooooooooth
        # rrt = RRTStarBidirectionalHeuristic(X, self.step, x_init, x_goal, self.maxIterNum, self.r, self.checkGoalProb, 32)
        # self.path = np.array(rrt.rrt_star_bid_h())

        # rrt = RRTStarBidirectional(X, self.step, x_init, x_goal, self.maxIterNum, self.r, self.checkGoalProb, 16)
        # self.path = np.array(rrt.rrt_star_bidirectional())
        
        rrt = RRTStar(X, self.step, x_init, x_goal, self.maxIterNum, self.r, self.checkGoalProb, 4)
        self.path = np.array(rrt.rrt_star())

        endTime = time.time()
        # rospy.loginfo("RTT finished: " + str(endTime - startTime))

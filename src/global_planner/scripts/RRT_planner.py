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
from std_msgs.msg import String
from actuator.msg import StateVector

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

        # 优化采样空间
        # subscribe human force
        rospy.Subscriber("stickSignal", String, self.humanCmd_callback, queue_size=1)
        self.thresholdForce = rospy.get_param("/shared_controller/threshold_force", 3.5)
        self.twoDimFlag = rospy.get_param("/two_dimesion", False)
        self.humanForce = np.zeros((3, 1))
        self.humanForce_valid = np.zeros((3, 1))

        # subscribe robot states from actuator
        rospy.Subscriber('/actuator/robotState', StateVector, self.cartesianState_callBack, queue_size=1)
        self.currentStates = np.zeros((6, 1))
        self.deviation = rospy.get_param("/shared_controller/deviation", 0.1)

    def humanCmd_callback(self, msg):
        posAndForce = msg.data.split(',')
        # two dimension setup
        if self.twoDimFlag:
            self.humanForce = np.array([-float(posAndForce[3]), -float(posAndForce[4]), 0])
        else:
            self.humanForce = np.array([-float(posAndForce[3]), float(posAndForce[5]), -float(posAndForce[4])])
        
        # only use valid force
        if np.linalg.norm(self.humanForce) > self.thresholdForce:
            self.humanForce_valid = self.humanForce
    
    def cartesianState_callBack(self, msg):
        self.currentStates[0] = msg.x
        self.currentStates[1] = msg.y
        self.currentStates[2] = msg.z
        self.currentStates[3] = msg.dx
        self.currentStates[4] = msg.dy
        self.currentStates[5] = msg.dz

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

        # shrink search space
        shrinkSpace = self.searchSpace.copy()
        if np.linalg.norm(self.humanForce_valid) > 0.001:
            rospy.loginfo("shrink search space")
            # shrink x dimension
            if self.humanForce_valid[0] > 0:
                shrinkSpace[0][0] = self.currentStates[0] - self.deviation
            elif self.humanForce_valid[0] < 0:
                shrinkSpace[0][1] = self.currentStates[0] + self.deviation
            # shrink y dimension
            if self.humanForce_valid[1] > 0:
                shrinkSpace[1][0] = self.currentStates[1] - self.deviation
            elif self.humanForce_valid[1] < 0:
                shrinkSpace[1][1] = self.currentStates[1] + self.deviation
            
            X = SearchSpace(shrinkSpace, self.normalizedObstacles)
        else:
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

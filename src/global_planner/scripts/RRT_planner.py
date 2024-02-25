#!/usr/bin/env python3
# This file is the implementation of the RRT planner

# when using launch file, must set sys.path.append(os.path.dirname(__file__)) to use classes in the same package
import sys
import os

sys.path.append(os.path.dirname(__file__))

from GlobalPlannerClass import GlobalPlanner
import numpy as np

from visualization_msgs.msg import Marker
import rospy


class RRTPlanner(GlobalPlanner):
    def __init__(self):
        super(RRTPlanner, self).__init__()
        self.optimizePath = None
        self.maxIterNum = None
        self.step = None
        self.searchSpace = None
        self.greedyProb = None

    def initPlanner(self):
        # RRT搜索步长
        self.step = rospy.get_param('/RRT_planner/step_size', 0.1)
        # RRT最大迭代步数
        self.maxIterNum = rospy.get_param('/RRT_planner/max_iter', 10000)
        # 搜索空间尺寸
        self.searchSpace = rospy.get_param('/RRT_planner/search_space', {'x': [-5, 5], 'y': [-5, 5], 'z': [-5, 5]})
        self.searchSpace = np.array([self.searchSpace['x'], self.searchSpace['y'], self.searchSpace['z']])
        assert self.searchSpace.shape == (3, 2)
        # 贪婪搜索概率
        self.greedyProb = rospy.get_param('/RRT_planner/greedy_prob', 0.1)
        self.optimizePath = rospy.get_param('/RRT_planner/optimize_path', False)

    # check if the point collides with the obstacles
    def pointCollisionDetection(self, point):
        # obstacle is a Marker message
        for obstacle in self.obstacles:
            if obstacle.type == Marker.SPHERE:
                # check if the point is in the sphere
                if np.linalg.norm(
                        np.array([obstacle.pose.position.x, obstacle.pose.position.y, obstacle.pose.position.z])
                        - point) <= obstacle.scale.x + self.obstacles_dilate:
                    return True

        return False

    # check if the line segment collides with the obstacles
    def lineCollisionDetection(self, p_s, p_e):
        if p_s.shape != (3, 1) or p_e.shape != (3, 1):
            raise Exception('The shape of startPoint and endPoint must be (3, 1).')

        checkVector = p_s - p_e
        checkStep = self.step / 5         # 5 is the number of check points in the line segment
        checkNum = int(np.linalg.norm(checkVector) / checkStep)

        for i in range(checkNum):
            checkPoint = p_s + checkVector * i / checkNum
            if self.pointCollisionDetection(checkPoint):
                return True

        return False

    def planPath(self):
        # RRT算法
        # optimize：是否进行路径优化
        startPoint = np.array([self.start.x, self.start.y, self.start.z])
        endPoint = np.array([self.goal.x, self.goal.y, self.goal.z])

        if self.pointCollisionDetection(startPoint):
            rospy.logerr("Start point is in the obstacle")

        iterTime = 0
        RRTTree = [[startPoint, -1]]

        while iterTime < self.maxIterNum:
            iterTime += 1

            # startTime = time.time()

            # 按概率生成随机点
            if np.random.rand() > self.greedyProb:
                randPoint = np.random.rand(3) * (self.searchSpace[:, 1] - self.searchSpace[:, 0]) + self.searchSpace[:, 0]
                # randPoint = randPoint.reshape((3, 1)) + startPoint
                randPoint = randPoint.reshape((3, 1))   # 使用绝对搜索空间
            else:
                randPoint = endPoint

            # endTime = time.time()
            # print("Generate:", endTime - startTime)
            # startTime = time.time()

            # 找到树上的最近点
            minDis = 1000000
            minIndex = -1
            for i in range(len(RRTTree)):
                dis = np.linalg.norm(randPoint - RRTTree[i][0])
                if dis < minDis:
                    minDis = dis
                    minIndex = i

            # 生成新点
            newPoint = RRTTree[minIndex][0] + (randPoint - RRTTree[minIndex][0]) / minDis * self.step

            # endTime = time.time()
            # print("Find near:", endTime - startTime)
            # startTime = time.time()

            # 检测新点是否与障碍物相交
            if self.lineCollisionDetection(RRTTree[minIndex][0], newPoint):
                continue

            # 将新点加入树
            RRTTree.append([newPoint, minIndex])

            # endTime = time.time()
            # print("check collision:", endTime - startTime)

            # 检测是否到达目标点
            startIndex = -1
            if np.linalg.norm(newPoint - endPoint) < self.goal_tolerance:
                # print("RRT search done")
                startIndex = minIndex
                break

            # debug
            # if iterTime % 500 == 0:
            #     print("Distance to end:", np.linalg.norm(newPoint - self.endPoint))
            #     print("RRT searching")

        if iterTime == self.maxIterNum:
            rospy.logerr("RRT search failed, max iteration reached")
            return

        # 从终点回溯到起点
        self.path = endPoint
        while startIndex != -1:
            self.path = np.hstack((RRTTree[startIndex][0], self.path))
            startIndex = RRTTree[startIndex][1]

        # DEBUG
        # if np.linalg.norm(self.path[:, 0].reshape(3, 1) - self.startPoint) == 0 and np.linalg.norm(
        #         self.path[:, -1].reshape(3, 1) - self.endPoint) == 0:
        #     print("RRT search done")

        # 路径优化
        if self.optimizePath:
            self.pathOptimization()

    def pathOptimization(self):
        # 贪婪优化，去除冗余点
        startIndex = 0
        endIndex = self.path.shape[1] - 1
        detectTimes = self.path.shape[1] - 1

        greedyPath = self.path[:, 0].reshape((3, 1))

        while detectTimes > 0:
            detectTimes -= 1
            startPoint = self.path[:, startIndex].reshape((3, 1))
            endPoint = self.path[:, endIndex].reshape((3, 1))

            if self.lineCollisionDetection(startPoint, endPoint):
                endIndex -= 1
            else:
                greedyPath = np.hstack((greedyPath, endPoint))
                startIndex = endIndex
                endIndex = self.path.shape[1] - 1
                detectTimes = self.path.shape[1] - 1 - startIndex

        # 设置点之间的最大距离，如果超过这个距离则等分起始点和终止点之间的距离，同时保持路径点的顺序
        extendPath = np.array([self.start.x, self.start.y, self.start.z])
        maxDistance = self.step * 3
        for i in range(greedyPath.shape[1] - 1):
            startPoint = greedyPath[:, i].reshape((3, 1))
            endPoint = greedyPath[:, i + 1].reshape((3, 1))
            if np.linalg.norm(endPoint - startPoint) > maxDistance:
                newPointNum = int(np.ceil(np.linalg.norm(endPoint - startPoint) / maxDistance))

                if newPointNum < 2:
                    raise Exception("Extend Path Error")

                newPointSet = startPoint + (endPoint - startPoint) / newPointNum
                for j in range(1, newPointNum - 1):
                    newPoint = startPoint + (endPoint - startPoint) / newPointNum * (j + 1)
                    newPointSet = np.hstack((newPointSet, newPoint))
                extendPath = np.hstack((extendPath, newPointSet, endPoint))
            else:
                extendPath = np.hstack((extendPath, endPoint))

        # self.path = greedyPath
        self.path = extendPath


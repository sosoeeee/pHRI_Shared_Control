#!/usr/bin/env python3
import math

import rospy
from visualization_msgs.msg import MarkerArray
from global_planner.srv import GlobalPlanning, GlobalPlanningResponse
import tf

# visualization for debugging
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import time
from abc import abstractmethod
import numpy as np


class BaseGlobalPlanner:
    def __init__(self):
        # env_obstacles
        self.obstacles = None
        self.env_subscriber = rospy.Subscriber("/env_obstacles", MarkerArray, self.updateObstacles, queue_size=1)
        self.tf_listener = tf.TransformListener()

        # add service to plan the path
        self.global_plan_service = rospy.Service('global_plan', GlobalPlanning, self.handle_GlobalPlanning)

        # initialize the planner
        self.obstacles_dilate = rospy.get_param('/global_planner/obstacles_dilate', 0.01)
        self.smooth_step = rospy.get_param('/global_planner/smooth_step', 0.1)
        self.world_frame = rospy.get_param('/world_frame', 'map')
        self.initPlanner()

        self.start = None         # type float32[3], shape is [x, y, z]
        self.goal = None          # type float32[3], shape is [x, y, z]
        self.path = None          # type np.ndarray, shape (N, 3)

        # visualisation for debugging
        # self.path_publisher = rospy.Publisher('global_path', Path, queue_size=1)
        # self.publish_rate = rospy.get_param('/global_planner/publish_rate', 10)
        # self.path_visual = None  # type nav_msgs/Path

    @abstractmethod
    def initPlanner(self):
        pass

    @abstractmethod
    def planPath(self):
        pass

    def smoothPath(self):
        step = np.sum((self.path[1] - self.path[0]) ** 2) ** 0.5
        if step > self.smooth_step:
            return self.path
        else:
            n = math.ceil(self.smooth_step / step)
            iterations = int((self.path.shape[0] - 2) / n)  # remove start and end point
            smoothPath = np.zeros((iterations, self.path.shape[1]))
            averageVector = np.ones((1, n)) * (1 / n)
            for i in range(iterations):
                smoothPath[i] = np.dot(averageVector, self.path[(1 + i*n):(i+1)*n + 1, :])
            # add start and end point
            smoothPath = np.vstack((self.path[0], smoothPath))
            smoothPath = np.vstack((smoothPath, self.path[-1]))

            rospy.loginfo("before smoothing points number: %d, after smoothing: %d" % (self.path.shape[0], smoothPath.shape[0]))

            return smoothPath

    def handle_GlobalPlanning(self, req):
        # get the request
        self.goal = req.goal
        self.start = req.start

        # wait until the env obstacles is ready
        if self.obstacles is None:
            time.sleep(0.1)
        
        # plan the path, self.path is (N, dimension)
        self.planPath()

        self.path = self.smoothPath()

        # return the response
        res = GlobalPlanningResponse()
        res.path = self.path.flatten().tolist()   # switch to float32[]
        return res

    def updateObstacles(self, obstacleSet):
        self.obstacles = obstacleSet.markers
        # transform obstacles into world frame
        for obstacle in self.obstacles:
            (trans, _) = self.tf_listener.lookupTransform(self.world_frame, obstacle.header.frame_id, rospy.Time(0))
            obstacle.pose.position.x += trans[0]
            obstacle.pose.position.y += trans[1]
            obstacle.pose.position.z += trans[2]

    # visualization for debugging
    # def Array2Pose(self, point):
    #     pose = PoseStamped()
    #     pose.header.frame_id = self.world_frame
    #     pose.header.stamp = rospy.Time.now()
    #     pose.pose.position.x = point[0]
    #     pose.pose.position.y = point[1]
    #     pose.pose.position.z = point[2]
    #     pose.pose.orientation.x = 0
    #     pose.pose.orientation.y = 0
    #     pose.pose.orientation.z = 0
    #     pose.pose.orientation.w = 1
    #     return pose

    def run(self):
        # visualization for debugging
        # while not rospy.is_shutdown():
        #     self.path_visual = Path()
        #     self.path_visual.header.frame_id = self.world_frame
        #     if self.path is not None:
        #         for point in self.path:
        #             self.path_visual.poses.append(self.Array2Pose(point))
        #
        #     self.path_publisher.publish(self.path_visual)
        #
        #     rospy.Rate(self.publish_rate).sleep()
        rospy.spin()


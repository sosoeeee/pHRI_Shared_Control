#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import MarkerArray
from global_planner.srv import GlobalPlanning, GlobalPlanningResponse

# visualization for debugging
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import time


class GlobalPlanner:
    def __init__(self):
        # env_obstacles
        self.obstacles = None
        self.env_subscriber = rospy.Subscriber("/env_obstacles", MarkerArray, self.updateObstacles, queue_size=1)

        # add service to plan the path
        self.global_plan_service = rospy.Service('global_plan', GlobalPlanning, self.handle_GlobalPlanning)

        # initialize the planner
        self.obstacles_dilate = rospy.get_param('/global_planner/obstacles_dilate', 0.01)
        self.goal_tolerance = rospy.get_param('/global_planner/goal_tolerance', 0.1)
        self.world_frame = rospy.get_param('/world_frame', 'map')
        self.initPlanner()

        self.start = None         # type float32[3], shape is [x, y, z]
        self.goal = None          # type float32[3], shape is [x, y, z]
        self.path = None          # type np.ndarray, shape (N, 3)

        # visualisation for debugging
        self.path_publisher = rospy.Publisher('global_path', Path, queue_size=1)
        self.publish_rate = rospy.get_param('/global_planner/publish_rate', 10)
        self.path_visual = None  # type nav_msgs/Path

    # can be a virtual function
    def initPlanner(self):
        pass

    # can be a virtual function
    def planPath(self):
        pass

    def handle_GlobalPlanning(self, req):
        # get the request
        self.goal = req.goal
        self.start = req.start

        # wait until the env obstacles is ready
        if self.obstacles is None:
            time.sleep(0.1)
        
        # plan the path
        self.planPath()

        # return the response
        res = GlobalPlanningResponse()
        res.path = self.path.flatten().tolist()   # switch to float32[]
        return res

    def updateObstacles(self, obstacleSet):
        self.obstacles = obstacleSet

    # visualization for debugging
    def Array2Pose(self, point):
        pose = PoseStamped()
        pose.header.frame_id = self.world_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = point[2]
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        return pose

    def run(self):
        # visualization for debugging
        while not rospy.is_shutdown():
            self.path_visual = Path()
            self.path_visual.header.frame_id = self.world_frame
            if self.path is not None:
                for point in self.path:
                    self.path_visual.poses.append(self.Array2Pose(point))

            self.path_publisher.publish(self.path_visual)
            rospy.sleep(self.publish_rate)


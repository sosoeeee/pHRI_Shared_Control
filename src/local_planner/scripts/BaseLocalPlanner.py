import numpy as np
import rospy
from global_planner.srv import *
from local_planner.srv import LocalPlanning, LocalPlanningResponse
from geometry_msgs.msg import Point

import time
from abc import abstractmethod


class BaseLocalPlanner:
    def __init__(self):
        # add service to plan the local trajectory
        self.local_planner_service = rospy.Service('/local_planner/local_plan', LocalPlanning, self.handle_LocalPlanning)

        # initialize the planner
        self.start_pos = None
        self.start_vel = None
        self.start_acc = None
        self.goal_pos = None
        self.goal_vel = None
        self.goal_acc = None
        self.max_vel = None
        self.max_acc = None
        self.total_time = None
        self.control_frequency = None

        self.refPath = None         # type float32[]
        self.global_plan_success = False

        self.initPlanner()

        # deubgging
        # self.rate = rospy.Rate(1)  # 1hz

    @abstractmethod
    def initPlanner(self):
        pass

    @abstractmethod
    def planTrajectory(self):
        pass

    def handle_LocalPlanning(self, req):
        # get the request
        self.start_pos = req.start_pos
        self.start_vel = req.start_vel
        self.start_acc = req.start_acc
        self.goal_pos = req.goal_pos
        self.goal_vel = req.goal_vel
        self.goal_acc = req.goal_acc
        self.max_vel = req.max_vel
        self.max_acc = req.max_acc
        self.total_time = req.total_time
        self.control_frequency = req.control_frequency

        # request the global path (block function)
        self.global_planner_client(self.start_pos, self.goal_pos)

        # plan the trajectory
        res = LocalPlanningResponse()
        
        # s = time.time()
        if self.global_plan_success:
            res.success = True
            res.trajectory = self.planTrajectory().T.flatten().tolist()
        else:
            res.success = False
            res.trajectory = []
        # e = time.time()
        # rospy.loginfo("Minisnap traj finished: " + str(e - s))
        # rospy.loginfo("traj length: %d" % (len(res.trajectory)/(len(self.start_pos)*2)))

        # res.path = self.path.flatten().tolist()  # switch to float32[]
        return res

    def global_planner_client(self, start, goal):
        rospy.wait_for_service('global_plan')
        try:
            plan_path = rospy.ServiceProxy('global_plan', GlobalPlanning)
            res = plan_path(start, goal)
            if res.success:
                self.refPath = res.path
                self.global_plan_success = True
            else:
                self.global_plan_success = False
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def run(self):
        # debugging
        # vertex1 = np.array([0.5, 0.5, 0.5])
        # vertex2 = np.array([7.5, 7.5, 7.5])
        # while not rospy.is_shutdown():
        #     # start = np.random.rand(3) * (vertex2 - vertex1) + vertex1
        #     # goal = np.random.rand(3) * (vertex2 - vertex1) + vertex1
        #     start = vertex1
        #     goal = vertex2
        #     rospy.loginfo("req planning")
        #     self.global_planner_client(start.tolist(), goal.tolist())
        #     # self.rate.sleep()
        #     time.sleep(2)
        rospy.spin()

    # def Array2Point(self, array):
    #     point = Point()
    #     point.x = array[0]
    #     point.y = array[1]
    #     point.z = array[2]
    #     return point

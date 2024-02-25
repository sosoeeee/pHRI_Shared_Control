import numpy as np
import rospy
from global_planner.srv import *
from geometry_msgs.msg import Point

import time

class LocalPlanner:
    def __init__(self):
        # add service to plan the local trajectory
        # self.local_planner_service = rospy.Service('local_planner', LocalPlanner, self.local_planner_callback)

        # deubgging
        # self.rate = rospy.Rate(1)  # 1hz
        pass

    def local_planner_callback(self, request):
        print("Received request: ", request)
        return True

    def global_planner_client(self, start, goal):
        rospy.wait_for_service('global_plan')
        try:
            plan_path = rospy.ServiceProxy('global_plan', GlobalPlanning)
            resp = plan_path(start, goal)
            return resp.path
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def run(self):
        # debugging
        vertex1 = np.array([0.5, 0.5, 0.5])
        vertex2 = np.array([7.5, 7.5, 7.5])
        while not rospy.is_shutdown():
            # start = np.random.rand(3) * (vertex2 - vertex1) + vertex1
            # goal = np.random.rand(3) * (vertex2 - vertex1) + vertex1
            start = vertex1
            goal = vertex2
            rospy.loginfo("req planning")
            path = self.global_planner_client(self.Array2Point(start), self.Array2Point(goal))
            # self.rate.sleep()
            time.sleep(2)

    def Array2Point(self, array):
        point = Point()
        point.x = array[0]
        point.y = array[1]
        point.z = array[2]
        return point

#! /usr/bin/env python3
import sys
import os

# Add the current directory to the path module
sys.path.append(os.path.dirname(__file__))

import numpy as np
import rospy
from BaseTaskServer import BaseTaskServer
import actionlib
from task_publisher.msg import ReachGoal
from task_publisher.msg import pubGoalAction, pubGoalFeedback, pubGoalResult


# visualization
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from controller.msg import VisualTraj

import time


class PubGoalActionServer(BaseTaskServer):
    # create messages that are used to publish feedback/result
    _feedback = pubGoalFeedback()
    _result = pubGoalResult()

    def __init__(self, name):
        super().__init__()
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, pubGoalAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._pubGoal = rospy.Publisher('/task/reachGoal', ReachGoal, queue_size=10)
        self._as.start()

        # recorded data (customize by yourself)
        self.realTraj = None

        # visual publisher
        self.vis_pubGoal = rospy.Publisher('/task/visual/goal', PointStamped, queue_size=1)
        self.vis_pubHuman = rospy.Publisher('/task/visual/humanForce', Marker, queue_size=1)
        self.vis_pubPos = rospy.Publisher('/task/visual/curPos', PointStamped, queue_size=1)
        self.vis_pubTraj = rospy.Publisher('/task/visual/Traj', Path, queue_size=1)

        # visual data
        self.world_frame = rospy.get_param('/world_frame', 'map')
        self.vis_goal = PointStamped()
        self.vis_goal.header.frame_id = self.world_frame

        self.vis_human = Marker()
        self.vis_human.header.frame_id = self.world_frame
        self.vis_human.action = Marker.ADD
        self.vis_human.ns = 'task'
        self.vis_human.type = Marker.ARROW
        self.vis_human.scale.x = 0.05
        self.vis_human.scale.y = 0.08
        self.vis_human.color.r = 0
        self.vis_human.color.g = 0
        self.vis_human.color.b = 1

        self.vis_curPos = PointStamped()
        self.vis_curPos.header.frame_id = self.world_frame

        rospy.Subscriber("/controller/globalTraj", VisualTraj, self.updateVis_traj, queue_size=1)
        self.vis_traj = Path()
        self.vis_traj.header.frame_id = self.world_frame

    def execute_cb(self, goal):
        # local variables
        controlFrequency = rospy.get_param("/controller/control_frequency", 10)
        r = rospy.Rate(controlFrequency)

        # initialize feedback and result msg
        self._feedback.distance_to_goal = 0
        self.realTraj = np.array([0, 0, 0]).reshape(3, 1)
        self._result.real_time_taken = 0

        # publish info to the console for the user
        rospy.loginfo('%s: Executing, the goal is (%.2f, %.2f, %.2f)' %
                      (self._action_name, goal.goal[0], goal.goal[1], goal.goal[2]))

        # publish goal to controller
        reachGoal = ReachGoal()
        reachGoal.goal = goal.goal
        reachGoal.time_taken = goal.time_taken
        reachGoal.tolerance = goal.tolerance
        self._pubGoal.publish(reachGoal)

        # initial goal
        self.vis_goal.point.x = goal.goal[0]
        self.vis_goal.point.y = goal.goal[1]
        self.vis_goal.point.z = goal.goal[2]

        startTime = time.time()

        # execute task
        while True:
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                break

            # update recorded data (used for data analysis)
            self.recordData()

            # update monitor data (used for visualization)
            self.updateInterface()

            # send feedback to client
            distanceToGoal = np.sum((self.currentStates - np.array(goal.goal).reshape(3, 1)) ** 2)
            self._feedback.distance_to_goal = distanceToGoal
            self._as.publish_feedback(self._feedback)

            # check end and send result to client
            if distanceToGoal < goal.tolerance:
                rospy.loginfo('%s: Completed' % self._action_name)
                endTime = time.time()
                self._result.real_time_taken = endTime - startTime
                self._feedback.trajectory = self.realTraj[:, 1:].T.flatten().tolist()
                self._as.set_succeeded(self._result)

            r.sleep()

    def updateInterface(self):
        # goal
        self.vis_pubGoal.publish(self.vis_goal)

        # human force
        self.vis_human.points = [Point(), Point()]
        self.vis_human.points[0].x = self.currentStates[0]
        self.vis_human.points[0].y = self.currentStates[1]
        self.vis_human.points[0].z = self.currentStates[2]
        self.vis_human.points[1].x = self.currentStates[0] + self.humanForce[0] * 0.05
        self.vis_human.points[1].y = self.currentStates[1] + self.humanForce[1] * 0.05
        self.vis_human.points[1].z = self.currentStates[2] + self.humanForce[2] * 0.05
        self.vis_pubHuman.publish(self.vis_human)

        # current position
        self.vis_curPos.point.x = self.currentStates[0]
        self.vis_curPos.point.y = self.currentStates[1]
        self.vis_curPos.point.z = self.currentStates[2]
        self.vis_pubPos.publish(self.vis_curPos)

        # planned trajectory (if controller have)
        self.vis_pubTraj.publish(self.vis_traj)

    def updateVis_traj(self, msg):
        pathArray = np.array(msg.trajectory).reshape(-1, msg.dimension)
        self.vis_traj.poses = []
        for point in pathArray:
            # rospy.loginfo("vis traj append point (%.2f, %.2f, %.2f)" % (point[0], point[1], point[2]))
            self.vis_traj.poses.append(self.Array2Pose(point))

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

    def recordData(self):
        self.realTraj = np.hstack((self.realTraj, self.currentStates)) # shape is (dim, N)

#! /usr/bin/env python3
import numpy as np
import rospy
from BaseTaskServer import BaseTaskServer
import actionlib
from task_publisher.action import *
from task_publisher.msg import ReachGoal

import time


class PubGoalActionServer(BaseTaskServer):
    # create messages that are used to publish feedback/result
    _feedback = pubGoalFeedback()
    _result = pubGoalResult()

    def __init__(self, name):
        super().__init__()
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, PubGoalAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._pubGoal = rospy.Publisher('/task/reachGoal', ReachGoal, queue_size=10)
        self._as.start()

    def execute_cb(self, goal):
        # helper variables
        controlFrequency = rospy.get_param("/controller/control_frequency", 10)
        r = rospy.Rate(controlFrequency)

        self._feedback.distance_to_goal = 0
        Trajectory = np.array([0, 0, 0])
        self._result.real_time_taken = 0

        # publish info to the console for the user
        rospy.loginfo('%s: Executing, the goal is (%d, %d, %d)' %
                      (self._action_name, goal.goal[0], goal.goal[1], goal.goal[2]))

        # publish goal to controller
        reachGoal = ReachGoal()
        reachGoal.goal = goal.goal
        reachGoal.max_time_taken = goal.max_time_taken
        reachGoal.tolerance = goal.tolerance
        self._pubGoal.publish(reachGoal)

        startTime = time.time()

        # execute task
        while True:
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                break

            # send feedback to client
            distanceToGoal = np.sum((self.currentStates - np.array(goal.goal).reshape(3, 1)) ** 2)
            self._feedback.distance_to_goal = distanceToGoal
            self._as.publish_feedback(self._feedback)

            Trajectory = np.hstack(Trajectory, self.currentStates)

            if distanceToGoal < goal.tolerance:
                rospy.loginfo('%s: Completed' % self._action_name)

                endTime = time.time()
                self._result.real_time_taken = endTime - startTime
                self._feedback.trajectory = Trajectory[:, 1:].flatten().tolist()
                self._as.set_succeeded(self._result)

            r.sleep()


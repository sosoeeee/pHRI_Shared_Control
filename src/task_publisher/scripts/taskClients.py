#!/usr/bin/env python3
import sys
import os

# Add the current directory to the path module

sys.path.append(os.path.dirname(__file__))

import rospy
import actionlib
from task_publisher.msg import pubGoalAction, pubGoalGoal
from task_publisher.msg import pubPathAction, pubPathGoal
import numpy as np


# instantiate a specific reaching goal task
class PubGoalActionClient:
    def __init__(self, args):
        self.client = actionlib.SimpleActionClient('ReachGoal', pubGoalAction)
        self.client.wait_for_server()

        # create a goal instance
        self.goal = pubGoalGoal()
        self.goal.goal.append(args['goal_x'])
        self.goal.goal.append(args['goal_y'])
        self.goal.goal.append(args['goal_z'])
        self.goal.time_taken = args['time_taken']
        self.goal.tolerance = args['tolerance']

        self._id = args['task_id']

        self.done = False

    def sendReq(self):
        self.client.send_goal(self.goal, done_cb=self.done_callback, feedback_cb=self.feedback_callback)

    # return true, if status is one of the terminal states
    def isDone(self):
        return self.done

    def done_callback(self, status, res):
        rospy.loginfo("ReachGoal task (id: %d) is done! Time cost %.2f" % (self._id, res.real_time_taken))

        # store data
        realTraj = np.array(res.trajectory).reshape(-1, 3)

        self.done = True

    def feedback_callback(self, feedback):
        pass


# instantiate a specific following path task
class PubPathActionClient:
    def __init__(self, args):
        self.client = actionlib.SimpleActionClient('FollowPath', pubPathAction)
        self.client.wait_for_server()

        # create a goal instance
        self.goal = pubGoalGoal()
        self.goal.end_point.append(args['goal_x'])
        self.goal.end_point.append(args['goal_y'])
        self.goal.end_point.append(args['goal_z'])
        self.goal.tolerance = args['tolerance']
        self.goal.file_path = args['file_path']

        self._id = args['task_id']

        self.done = False

    def sendReq(self):
        self.client.send_goal(self.goal, done_cb=self.done_callback, feedback_cb=self.feedback_callback)

    # return true, if status is one of the terminal states
    def isDone(self):
        return self.done

    def done_callback(self, status, res):
        rospy.loginfo("FollowPath task (id: %d) is done!")

        # store data

        self.done = True

    def feedback_callback(self, feedback):
        pass
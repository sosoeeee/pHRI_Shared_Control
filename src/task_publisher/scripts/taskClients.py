#!/usr/bin/env python3
import sys
import os

# Add the current directory to the path module
sys.path.append(os.path.dirname(__file__))

import rospy
import actionlib
from task_publisher.msg import pubGoalAction, pubGoalGoal


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
        # rospy.loginfo("ReachGoal task (id: %d) result is: " + res)
        rospy.loginfo("Goal status is %d" % status)

        # store data

        self.done = True

    def feedback_callback(self, feedback):
        pass

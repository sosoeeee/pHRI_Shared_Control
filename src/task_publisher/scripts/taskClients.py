#!/usr/bin/env python3
import rospy
import actionlib
from task_publisher.action import *


# instantiate a specific reaching goal task
class PubGoalActionClient:
    def __init__(self, args):
        self.client = actionlib.SimpleActionClient('ReachGoal', PubGoalAction)
        self.client.wait_for_server()

        # create a goal instance
        self.goal = PubGoalGoal()
        self.goal.goal[0] = args['goal_x']
        self.goal.goal[1] = args['goal_y']
        self.goal.goal[2] = args['goal_z']
        self.goal.max_time_taken = args['time_taken']
        self.goal.tolerance = args['tolerance']

        self._id = args['task_id']

        self.done = False

    def sendReq(self):
        self.client.send_goal(self.goal, done_cb=self.done_callback(), feedback_cb=self.feedback_callback())

    # return true, if status is one of the terminal states
    def isDone(self):
        return self.done

    def done_callback(self):
        res = self.client.get_result()
        rospy.loginfo("ReachGoal task (id: %d) result is: " + res)

        # store data

        self.done = True

    def feedback_callback(self):
        pass

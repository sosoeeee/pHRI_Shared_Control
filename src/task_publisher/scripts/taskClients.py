#!/usr/bin/env python3
import sys
import os

# Add the current directory to the path module

sys.path.append(os.path.dirname(__file__))

import rospy
import actionlib
from task_publisher.msg import pubGoalAction, pubGoalGoal
from task_publisher.msg import pubPathAction, pubPathGoal
from task_publisher.msg import pubTrajAction, pubTrajGoal
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

        # data collection
        self.robotTrajSet = []

    def sendReq(self):
        self.client.send_goal(self.goal, done_cb=self.done_callback, feedback_cb=self.feedback_callback)

    # return true, if status is one of the terminal states
    def isDone(self):
        return self.done

    def done_callback(self, status, res):
        rospy.loginfo("ReachGoal task (id: %d) is done! Time cost %.2f" % (self._id, res.real_time_taken))

        # store data
        controller_type = rospy.get_param("/controller_type", "Impedance")
        actualTraj = np.array(res.actualTraj).reshape((-1, 6))
        
        # because we need launch nodes by roslaunch, so we have to use absolute path here
        directory = "/home/jun/pHRI_Shared_Control/src/task_publisher/data/%s" % controller_type
        if not os.path.exists(directory):
            os.makedirs(directory)

        np.savetxt("/home/jun/pHRI_Shared_Control/src/task_publisher/data/%s/actualTraj_id%d_timecost_%.3f.txt" %
                    (controller_type, self._id, res.real_time_taken), actualTraj)
        k = 1
        for traj in self.robotTrajSet:
            np.savetxt("/home/jun/pHRI_Shared_Control/src/task_publisher/data/%s/robotTraj_id%d_%d_%.3f" %
                        (controller_type, self._id, k, traj['time']), traj['path'])
            k += 1
        self.done = True

    def feedback_callback(self, feedback):
        # default traj dimension is three :(
        traj = {'path': np.array(feedback.robotTraj).reshape((-1, 3)), 'time': feedback.time}
        self.robotTrajSet.append(traj)


# instantiate a specific following path task
class PubPathActionClient:
    def __init__(self, args):
        self.client = actionlib.SimpleActionClient('FollowPath', pubPathAction)
        self.client.wait_for_server()

        # create a goal instance
        self.goal = pubPathGoal()
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
        rospy.loginfo("FollowPath task (id: %d) is done!" % self._id)

        # store data
        controller_type = rospy.get_param("/controller_type", "Impedance")

        np.savetxt("/home/jun/pHRI_Shared_Control/src/task_publisher/data/%s/pathError_id%d.txt" %
                   (controller_type, self._id), np.array(res.reach_error).reshape((-1, 1)))
        np.savetxt("/home/jun/pHRI_Shared_Control/src/task_publisher/data/%s/pathRealTimeError_id%d.txt" %
                   (controller_type, self._id), np.array(res.real_time_error).reshape((-1, 1)))
        np.savetxt("/home/jun/pHRI_Shared_Control/src/task_publisher/data/%s/humanForce_id%d.txt" %
                   (controller_type, self._id), np.array(res.human_force).reshape((-1, 3)))

        self.done = True

    def feedback_callback(self, feedback):
        pass


# instantiate a specific following traj task
class PubTrajActionClient:
    def __init__(self, args):
        self.client = actionlib.SimpleActionClient('FollowTraj', pubTrajAction)
        self.client.wait_for_server()

        # create a goal instance
        self.goal = pubTrajGoal()
        self.goal.sample_frequency = args['sample_frequency']
        self.goal.file_path = args['file_path']

        self._id = args['task_id']

        self.done = False

    def sendReq(self):
        self.client.send_goal(self.goal, done_cb=self.done_callback, feedback_cb=self.feedback_callback)

    # return true, if status is one of the terminal states
    def isDone(self):
        return self.done

    def done_callback(self, status, res):
        rospy.loginfo("FollowPath task (id: %d) is done!" % self._id)

        # store data
        controller_type = rospy.get_param("/controller_type", "Impedance")

        np.savetxt("/home/jun/pHRI_Shared_Control/src/task_publisher/data/%s/TrajAvrError_id%d.txt" %
                   (controller_type, self._id), res.average_error)
        np.savetxt("/home/jun/pHRI_Shared_Control/src/task_publisher/data/%s/TrajRealTimeError_id%d.txt" %
                   (controller_type, self._id), np.array(res.real_time_error).reshape((-1, 1)))
        np.savetxt("/home/jun/pHRI_Shared_Control/src/task_publisher/data/%s/humanForce_id%d.txt" %
                   (controller_type, self._id), np.array(res.human_force).reshape((-1, 3)))

        self.done = True

    def feedback_callback(self, feedback):
        pass
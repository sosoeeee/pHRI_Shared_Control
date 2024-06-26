#! /usr/bin/env python3
import sys
import os

# Add the current directory to the path module
sys.path.append(os.path.dirname(__file__))

import numpy as np
import rospy
import rospkg
from BaseTaskServer import BaseTaskServer
import actionlib
from task_publisher.msg import ReachGoal
from task_publisher.msg import pubGoalAction, pubGoalFeedback, pubGoalResult
from task_publisher.msg import pubPathAction, pubPathFeedback, pubPathResult
from task_publisher.msg import pubTrajAction, pubTrajFeedback, pubTrajResult
from controller.msg import StateCmd

# visualization
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from controller.msg import VisualTraj
import tf

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
        self.startTime = None

        # recorded data (customize by yourself)
        self.data_actualTraj = None

        # visual publisher
        self.vis_pubGoal = rospy.Publisher('/task/visual/goal', Marker, queue_size=1)
        self.vis_pubHuman = rospy.Publisher('/task/visual/humanForce', Marker, queue_size=1)
        self.vis_pubPos = rospy.Publisher('/task/visual/curPos', PointStamped, queue_size=1)
        self.vis_pubTraj = rospy.Publisher('/task/visual/Traj', Path, queue_size=1)
        self.vis_pubRobotDesPos = rospy.Publisher('/task/visual/robotDesPos', PointStamped, queue_size=1)
        self.br = tf.TransformBroadcaster()
        self.q = [0, 0, 0, 1]

        # visual data
        self.world_frame = rospy.get_param('/world_frame', 'map')
        self.vis_goal = Marker()
        self.vis_goal.header.frame_id = self.world_frame
        self.vis_goal.action = Marker.ADD
        self.vis_goal.ns = 'task'
        self.vis_goal.type = Marker.SPHERE
        self.vis_goal.scale.x = 0.04
        self.vis_goal.scale.y = 0.04
        self.vis_goal.scale.z = 0.04
        self.vis_goal.color.r = 1
        self.vis_goal.color.g = 1
        self.vis_goal.color.b = 0
        self.vis_goal.color.a = 0.5

        self.vis_human = Marker()
        self.vis_human.header.frame_id = self.world_frame
        self.vis_human.action = Marker.ADD
        self.vis_human.ns = 'task'
        self.vis_human.type = Marker.ARROW
        self.vis_human.scale.x = 0.01
        self.vis_human.scale.y = 0.03
        self.vis_human.color.r = 1
        self.vis_human.color.g = 1
        self.vis_human.color.b = 0
        self.vis_human.color.a = 1
        self.thresholdForce = rospy.get_param("/shared_controller/threshold_force", 3.5)

        self.vis_curPos = PointStamped()
        self.vis_curPos.header.frame_id = self.world_frame

        rospy.Subscriber("/controller/robotDesPos", Point, self.updateVis_robotDesPos, queue_size=1)
        self.vis_robotDesPos = PointStamped()
        self.vis_robotDesPos.header.frame_id = self.world_frame

        rospy.Subscriber("/controller/globalTraj", VisualTraj, self.updateVis_traj, queue_size=1)
        self.vis_traj = Path()
        self.vis_traj.header.frame_id = self.world_frame

    def execute_cb(self, goal):
        # local variables
        controlFrequency = rospy.get_param("/controller/control_frequency", 10)
        r = rospy.Rate(controlFrequency)

        # initialize feedback and result msg
        self.data_actualTraj = np.zeros((6, 1))
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
        self.vis_goal.pose.position.x = goal.goal[0]
        self.vis_goal.pose.position.y = goal.goal[1]
        self.vis_goal.pose.position.z = goal.goal[2]

        self.startTime = time.time()

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

            # check end and send result to client
            distanceToGoal = np.sum((self.currentStates[:3, :] - np.array(goal.goal).reshape((3, 1))) ** 2) ** 0.5
            if distanceToGoal < goal.tolerance:
                rospy.loginfo('%s: Completed' % self._action_name)
                endTime = time.time()
                self._result.real_time_taken = endTime - self.startTime
                self._result.actualTraj = self.data_actualTraj[:, 1:].T.flatten().tolist()
                self._as.set_succeeded(self._result)
                break

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

        if np.linalg.norm(self.humanForce) > self.thresholdForce:
            self.vis_human.color.g = 0
        else:
            self.vis_human.color.g = 1

        self.vis_pubHuman.publish(self.vis_human)

        # current position
        self.vis_curPos.point.x = self.currentStates[0]
        self.vis_curPos.point.y = self.currentStates[1]
        self.vis_curPos.point.z = self.currentStates[2]
        self.vis_pubPos.publish(self.vis_curPos)

        if np.linalg.norm(self.humanForce) != 0:
            self.q = self.getQuaternion([1, 0, 0], self.humanForce[:3].flatten())
            # rospy.loginfo("quaternion: (%.2f, %.2f, %.2f, %.2f)" % (self.q [0], self.q [1], self.q [2], self.q [3]))

        self.br.sendTransform((self.vis_curPos.point.x, self.vis_curPos.point.y, self.vis_curPos.point.z),
                              (self.q[0], self.q[1], self.q[2], self.q[3]),
                              rospy.Time.now(),
                              "camera",
                              self.world_frame)

        # planned trajectory (if controller have)
        self.vis_pubTraj.publish(self.vis_traj)

        # robot desire pos
        self.vis_pubRobotDesPos.publish(self.vis_robotDesPos)

    def recordData(self):
        self.data_actualTraj = np.hstack((self.data_actualTraj, self.currentStates))  # shape is (2*dim, N)

    def updateVis_traj(self, msg):
        pathArray = np.array(msg.trajectory).reshape((-1, msg.dimension))
        self.vis_traj.poses = []
        for point in pathArray:
            # rospy.loginfo("vis traj append point (%.2f, %.2f, %.2f)" % (point[0], point[1], point[2]))
            self.vis_traj.poses.append(self.Array2Pose(point))

        # send feedback to clients
        self._feedback.robotTraj = msg.trajectory
        self._feedback.time = time.time() - self.startTime
        self._as.publish_feedback(self._feedback)

    def updateVis_robotDesPos(self, msg):
        self.vis_robotDesPos.point.x = msg.x
        self.vis_robotDesPos.point.y = msg.y
        self.vis_robotDesPos.point.z = msg.z

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

    def getQuaternion(self, ori_vec, loc_vec):
        ori_vec = np.array(ori_vec)
        loc_vec = np.array(loc_vec)
        ori_vec = ori_vec / np.linalg.norm(ori_vec)
        loc_vec = loc_vec / np.linalg.norm(loc_vec)
        axis = np.cross(ori_vec, loc_vec)
        angle = np.arccos(np.dot(ori_vec, loc_vec))
        return tf.transformations.quaternion_about_axis(angle, axis)


class PubPathActionServer(BaseTaskServer):
    # create messages that are used to publish feedback/result
    _feedback = pubPathFeedback()
    _result = pubPathResult()

    def __init__(self, name):
        super().__init__()
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, pubPathAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        # recorded data (customize by yourself)
        self.data_reachError = None
        self.data_realTimeError = None
        self.data_humanForce = None
        self.pathPoints = None

        # visual publisher
        self.vis_pubPath = rospy.Publisher('/task/visual/followPath', Marker, queue_size=1)

        # visual data
        self.world_frame = rospy.get_param('/world_frame', 'map')

        self.vis_PathPoints = Marker()
        self.vis_PathPoints.header.frame_id = self.world_frame
        self.vis_PathPoints.ns = "task"
        self.vis_PathPoints.type = Marker.SPHERE_LIST
        self.vis_PathPoints.action = Marker.ADD
        self.vis_PathPoints.scale.x = 0.02
        self.vis_PathPoints.scale.y = 0.02
        self.vis_PathPoints.color.r = 0
        self.vis_PathPoints.color.g = 1
        self.vis_PathPoints.color.b = 0
        self.vis_PathPoints.color.a = 1

    def execute_cb(self, goal):
        # local variables
        controlFrequency = rospy.get_param("/controller/control_frequency", 10)
        r = rospy.Rate(controlFrequency)  # update rate

        # initialize feedback and result msg
        rospack = rospkg.RosPack()
        self.pathPoints = np.loadtxt(rospack.get_path('task_publisher') + '/' + goal.file_path)
        if self.pathPoints.shape[1] != 3:
            raise Exception("check your path points txt file! Each line only contains one point")
        endPoint = np.array(goal.end_point).reshape((3, 1))  # generally same as goal in task "ReachGoal"
        self._feedback.distance_to_path = 0
        self.data_reachError = [np.inf for _ in range(self.pathPoints.shape[0])]
        self.data_realTimeError = []
        self.data_humanForce = np.zeros((3, 1))

        # publish info to the console for the user
        rospy.loginfo('%s: Executing, the end point is (%.2f, %.2f, %.2f)' %
                      (self._action_name, goal.end_point[0], goal.end_point[1], goal.end_point[2]))

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
            distanceToEnd = np.sum((self.currentStates[:3, :] - endPoint) ** 2) ** 0.5
            # the minimum among the distances from the current point to all points on the path.
            self._feedback.distance_to_path = np.min(self.data_reachError)
            self._as.publish_feedback(self._feedback)

            # check end and send result to client
            if distanceToEnd < goal.tolerance:
                rospy.loginfo('%s: Completed' % self._action_name)
                self._result.reach_error = self.data_reachError
                self._result.human_force = self.data_humanForce.T.flatten().tolist()
                self._result.real_time_error = self.data_realTimeError
                self._as.set_succeeded(self._result)
                break

            r.sleep()

    def updateInterface(self):
        # path point
        self.vis_PathPoints.points = []
        for point in self.pathPoints:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            self.vis_PathPoints.points.append(p)
        self.vis_pubPath.publish(self.vis_PathPoints)

    def recordData(self):
        # collect interact force
        self.data_humanForce = np.hstack((self.data_humanForce, self.humanForce))  # shape is (dim, N)
        # compute error in real time
        miniError = np.inf
        for i in range(len(self.data_reachError)):
            error = np.linalg.norm(self.currentStates[:3, :] - self.pathPoints[i].reshape((3, 1)))
            if error < self.data_reachError[i]:
                self.data_reachError[i] = error
            if error < miniError:
                miniError = error
        self.data_realTimeError.append(miniError)


class PubTrajActionServer(BaseTaskServer):
    # create messages that are used to publish feedback/result
    _feedback = pubTrajFeedback()
    _result = pubTrajResult()

    def __init__(self, name):
        super().__init__()
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, pubTrajAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        # recorded data (customize by yourself)
        self.data_sumError = None
        self.data_realTimeError = None
        self.data_humanForce = None
        self.refTraj = None
        self.idx = 0
        # wait robot start to move
        rospy.Subscriber('/nextState', StateCmd, self.controlCmd_callback, queue_size=1)
        self.startTask = False

        # visual publisher
        self.vis_pubFollowPoint = rospy.Publisher('/task/visual/followPoint', Marker, queue_size=1)
        self.vis_pubRefTraj = rospy.Publisher('/task/visual/refTraj', Path, queue_size=1)
        self.firstPub = True

        # visual data
        self.world_frame = rospy.get_param('/world_frame', 'map')

        self.vis_followPoint = Marker()
        self.vis_followPoint.header.frame_id = self.world_frame
        self.vis_followPoint.ns = "task"
        self.vis_followPoint.type = Marker.CUBE
        self.vis_followPoint.action = Marker.ADD
        self.vis_followPoint.scale.x = 0.02
        self.vis_followPoint.scale.y = 0.02
        self.vis_followPoint.scale.z = 0.02
        self.vis_followPoint.color.r = 0
        self.vis_followPoint.color.g = 1
        self.vis_followPoint.color.b = 0
        self.vis_followPoint.color.a = 1
        self.vis_followPoint.pose.orientation.x = 0.0
        self.vis_followPoint.pose.orientation.y = 0.0
        self.vis_followPoint.pose.orientation.z = 0.0
        self.vis_followPoint.pose.orientation.w = 1.0

        self.vis_refTraj = Path()
        self.vis_refTraj.header.frame_id = self.world_frame

    def execute_cb(self, goal):
        # local variables
        r = rospy.Rate(goal.sample_frequency)  # update rate

        # initialize feedback and result msg
        rospack = rospkg.RosPack()
        self.refTraj = np.loadtxt(rospack.get_path('task_publisher') + '/' + goal.file_path)
        if self.refTraj.shape[1] == 6:
            self.refTraj = self.refTraj[:, :3]
        length = self.refTraj.shape[0]
        if self.refTraj.shape[1] != 3:
            rospy.logerr("check your trajectory txt file! Each line only contains one point")
        self._feedback.current_error = 0
        self.data_sumError = 0
        self.data_realTimeError = []
        self.data_humanForce = np.zeros((3, 1))

        while self.startTask is False:
            rospy.sleep(0.01)

        # publish info to the console for the user
        rospy.loginfo('%s: Executing' % self._action_name)

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
            error = np.linalg.norm(self.currentStates[:3, :] - self.refTraj[self.idx].reshape((3, 1)))
            # the minimum among the distances from the current point to all points on the path.
            self._feedback.current_error = error
            self._as.publish_feedback(self._feedback)

            self.idx += 1

            # check end and send result to client
            if self.idx == length:
                rospy.loginfo('%s: Completed' % self._action_name)
                self._result.average_error = self.data_sumError / length
                self._result.real_time_error = self.data_realTimeError
                self._result.human_force = self.data_humanForce.T.flatten().tolist()
                self._as.set_succeeded(self._result)
                break

            r.sleep()

    def updateInterface(self):
        # follow point
        self.vis_followPoint.pose.position.x = self.refTraj[self.idx, 0]
        self.vis_followPoint.pose.position.y = self.refTraj[self.idx, 1]
        self.vis_followPoint.pose.position.z = self.refTraj[self.idx, 2]
        self.vis_pubFollowPoint.publish(self.vis_followPoint)

        # ref Traj
        self.vis_refTraj.poses = []
        for point in self.refTraj:
            self.vis_refTraj.poses.append(self.Array2Pose(point))
        self.vis_pubRefTraj.publish(self.vis_refTraj)

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
        # collect interact force
        self.data_humanForce = np.hstack((self.data_humanForce, self.humanForce))  # shape is (dim, N)
        # compute error in real time
        error = np.linalg.norm(self.currentStates[:3, :] - self.refTraj[self.idx].reshape((3, 1)))
        self.data_sumError += error
        self.data_realTimeError.append(error)

    def controlCmd_callback(self, msg):
        if self.startTask is False:
            self.startTask = True
        else:
            pass
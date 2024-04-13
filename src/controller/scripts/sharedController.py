#!/usr/bin/env python3
import math
import sys
import os

# Add the current directory to the path module, in order to import the LocalPlannerClass
sys.path.append(os.path.dirname(__file__))

import rospy
import rospkg
from BaseController import BaseController
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
# debug
# from nav_msgs.msg import Path
import tf
from local_planner.srv import *
from controller.msg import VisualTraj
from controller.msg import StateCmd

import numpy as np
import time


class SharedController(BaseController):
    def __init__(self):
        super(SharedController, self).__init__()

        # controller parameters
        # Impedance Model
        self.Md = None
        self.Cd = None

        # discrete state space model
        self.Ad = None
        self.Brd = None
        self.Bhd = None

        # local desired trajectory
        self.localLen = None  # predicted local trajectory length used by MPC
        self.robotLocalTraj = None  # 3*localLen
        self.humanLocalTraj = None  # 3*localLen

        # global robot desired trajectory
        self.computeGlobalTraj = False
        self.robotGlobalTraj = None
        self.robotGlobalTrajLen = None
        self.ori_robotGlobalTrajLen = None
        self.vis_pubRawTraj = rospy.Publisher('/controller/globalTraj', VisualTraj, queue_size=10)
        self.loadTraj = False

        # predicted safety index
        self.lambda_ = 1
        rospy.Subscriber("/env_obstacles", MarkerArray, self.updateObstacles, queue_size=1)
        self.world_frame = rospy.get_param('/world_frame', 'map')
        self.tf_listener = tf.TransformListener()
        self.obstacles = None
        self.obstaclesPoints = None

        # MPC params
        self.weight_r = None
        self.weight_h = None
        self.weight_tracking = None
        self.Qr = None
        self.Qh = None
        self.theta_rg = None
        self.theta_hg = None
        self.phi_g = None

        # the intensity of human intent
        # 0：follow robot
        # 1: change local traj
        # 2: change global traj
        self.humanIntent = 0
        self.thresholdForce = None

        # params for trajectory re-planning
        self.replanLen = None
        self.R = None
        self.replanPathNum = None  # the size of locally feasible trajectories set
        self.alpha = None  # weight param in trajectory energy function
        self.replanFreq = None  # control frequency must be the multiples of re-plan frequency
        self.ctr = 0
        self.replanTimeOut = None

        self.curIdx = -1
        self.pubRobotDesPos = rospy.Publisher('/controller/robotDesPos', Point, queue_size=1)

        self.loadParams()

        # debug
        # self.vis_pubLocalTraj_h = rospy.Publisher('/controller/Traj_h', Path, queue_size=1)
        # self.vis_pubLocalTraj_r = rospy.Publisher('/controller/Traj_r', Path, queue_size=1)

    def reInitial(self):
        # local desired trajectory
        self.robotLocalTraj = None  # 3*localLen
        self.humanLocalTraj = None  # 3*localLen

        # global robot desired trajectory
        self.computeGlobalTraj = False
        self.robotGlobalTraj = None
        self.robotGlobalTrajLen = None
        self.ori_robotGlobalTrajLen = None

        # predicted safety index
        self.lambda_ = None
        self.obstacles = None
        self.obstaclesPoints = None

        # the intensity of human intent
        self.humanIntent = 0

        # params for trajectory re-planning
        self.ctr = 0

        self.curIdx = -1

    def loadParams(self):
        # read parameters
        self.Md = rospy.get_param("/shared_controller/M", 1)
        self.Cd = rospy.get_param("/shared_controller/C", 1)
        self.localLen = rospy.get_param("/shared_controller/local_len", 50)
        self.thresholdForce = rospy.get_param("/shared_controller/threshold_force", 3.5)
        self.replanLen = rospy.get_param("/shared_controller/replan_len", 250)
        self.replanPathNum = rospy.get_param("/shared_controller/replan_len_num", 10)
        self.replanFreq = rospy.get_param("/shared_controller/replan_freq", 1)
        self.replanTimeOut = rospy.get_param("/shared_controller/replan_timeout", 0.5)
        self.alpha = rospy.get_param("/shared_controller/alpha", 100)
        self.weight_h = rospy.get_param("/shared_controller/weight_h", 1)
        self.weight_r = rospy.get_param("/shared_controller/weight_r", 10)
        self.weight_tracking = rospy.get_param("/shared_controller/weight_tracking", 10000)
        self.loadTraj = rospy.get_param("/shared_controller/load_traj", False)
        self.deviation = rospy.get_param("/shared_controller/deviation", 0.1)

        # state space model
        # Md_inv = np.linalg.inv(self.Md * np.eye(3))
        # A = np.zeros((6, 6))
        # A[0:3, 3:6] = np.eye(3)
        # A[3:6, 3:6] = -Md_inv.dot(self.Cd * np.eye(3))
        # Br = np.zeros((6, 3))
        # Br[3:6, :] = Md_inv
        # Bh = np.zeros((6, 3))
        # Bh[3:6, :] = Md_inv
        # C = np.zeros((3, 6))
        # C[:, 0:3] = np.eye(3)

        # T = 1 / self.controlFrequency
        # self.Ad = np.eye(6) + A * T
        # self.Brd = Br * T
        # self.Bhd = Bh * T
        # transition matrix
        T = 1 / self.controlFrequency
        tmp1 = math.exp(- self.Cd * T / self.Md)
        tmp2 = 1 - tmp1
        self.Ad = np.zeros((6, 6))
        self.Ad[0:3, 0:3] = np.eye(3)
        self.Ad[0:3, 3:6] = np.eye(3) * self.Md * tmp2 / self.Cd
        self.Ad[3:6, 3:6] = np.eye(3) * tmp1
        self.Bhd = np.zeros((6, 3))
        self.Bhd[0:3, :] = np.eye(3) * (T / self.Cd - self.Md / (self.Cd ** 2) * tmp2)
        self.Bhd[3:6, :] = np.eye(3) * tmp2 / self.Cd
        self.Brd = self.Bhd
        C = np.zeros((3, 6))
        C[:, 0:3] = np.eye(3)

        # 控制器参数设置
        phi = np.zeros((3 * self.localLen, 6))
        theta_h = np.zeros((3 * self.localLen, 3 * self.localLen))
        theta_r = np.zeros((3 * self.localLen, 3 * self.localLen))
        phi_row = C.dot(self.Ad)
        Qii_r = np.eye(3) * self.weight_r * self.weight_tracking
        Qii_h = np.eye(3) * self.weight_h * self.weight_tracking
        self.Qr = np.zeros((3 * self.localLen, 3 * self.localLen))
        self.Qh = np.zeros((3 * self.localLen, 3 * self.localLen))

        for i in range(self.localLen):
            phi[(i * 3):(3 * i + 3), :] = phi_row
            self.Qr[(i * 3):(3 * i + 3), (i * 3):(3 * i + 3)] = Qii_r
            self.Qh[(i * 3):(3 * i + 3), (i * 3):(3 * i + 3)] = Qii_h
            theta_h_row = np.eye(6)
            for j in range(i + 1):
                theta_h[(i * 3):(3 * i + 3), (3 * (i - j)):(3 * (i - j) + 3)] = np.dot(C, theta_h_row.dot(self.Bhd))
                theta_r[(i * 3):(3 * i + 3), (3 * (i - j)):(3 * (i - j) + 3)] = np.dot(C, theta_h_row.dot(self.Brd))
                theta_h_row = theta_h_row.dot(self.Ad)
            phi_row = np.dot(phi_row, self.Ad)

        self.theta_hg = np.vstack((theta_h, theta_h))
        self.theta_rg = np.vstack((theta_r, theta_r))
        self.phi_g = np.vstack((phi, phi))

        # initialize params for trajectory re-planning
        lenReplanA = self.replanLen
        replanA = np.zeros((lenReplanA + 3, lenReplanA))
        for i in range(1, lenReplanA + 3):
            for j in range(1, lenReplanA):
                if i == j:
                    replanA[i - 1, j - 1] = 1
                elif i == j + 1:
                    replanA[i - 1, j - 1] = -3
                elif i == j + 2:
                    replanA[i - 1, j - 1] = 3
                elif i == j + 3:
                    replanA[i - 1, j - 1] = -1
        self.R = replanA.T.dot(replanA)

    def computeCmd(self):
        # copy sensor data

        # print('---------')
        # s = time.time()

        humCmd = self.humanCmd.copy()
        curStates = self.currentStates.copy() ## ? strange, causing sensor data lag
        self.curIdx += 1

        if self.computeGlobalTraj is False:
            # compute global trajectory
            self.planGlobalTraj(curStates)

        if self.curIdx == self.robotGlobalTrajLen - self.replanLen:
            self.extendGlobalTraj()

        # e = time.time()
        # print('part1:' , e-s)
        # s = time.time()

        self.updateHumanLocalTraj(self.curIdx, humCmd, curStates)

        # e = time.time()
        # print('part1-human:' , e-s)
        # s = time.time()

        self.computeLambda(curStates)

        # e = time.time()
        # print('part3-lambda:' , e-s)
        # s = time.time()

        # self.ctr += 1
        # if self.ctr > self.controlFrequency / self.replanFreq and self.humanIntent == 2:
        #     self.ctr = 0
        #     self.changeGlobalTraj(self.curIdx, humCmd)

        # trigger re-planning until interaction force exceed threshold and last for '1/self.replanFreq'
        if self.humanIntent == 2:
            self.ctr += 1
            if self.ctr > self.controlFrequency / self.replanFreq:
                self.changeGlobalTraj(self.curIdx, self.humanCmd)
                self.ctr = 0
        else:
            self.ctr = 0
        
        robotDesPos = Point()
        robotDesPos.x = self.robotGlobalTraj[0, self.curIdx]
        robotDesPos.y = self.robotGlobalTraj[1, self.curIdx]
        robotDesPos.z = self.robotGlobalTraj[2, self.curIdx]
        self.pubRobotDesPos.publish(robotDesPos)

        # e = time.time()
        # print('part4-pub:' , e-s)
        # s = time.time()

        # rospy.loginfo("idx: %d" % self.curIdx)

        cmd = self.computeLocalTraj(self.curIdx, humCmd, curStates)

        # e = time.time()
        # print('part5-cmd:' , e-s)
        # s = time.time()

        return cmd

    def updateObstacles(self, obstacleSet):
        self.obstacles = obstacleSet.markers
        self.obstaclesPoints = None
        # transform obstacles into world frame
        for obstacle in self.obstacles:
            ori_pose = PoseStamped()
            ori_pose.pose = obstacle.pose
            ori_pose.header = obstacle.header
            tar_pose = self.tf_listener.transformPose(self.world_frame, ori_pose)
            obstacle.pose = tar_pose.pose

        # generate discrete points to represent obstacles, really low efficiency :(
        sample_step = 0.03
        for obstacle in self.obstacles:
            if self.obstaclesPoints is None:
                if obstacle.type == Marker.CUBE:
                    self.obstaclesPoints = self.generateCuboidPoints([obstacle.pose.position.x - obstacle.scale.x / 2,
                                                                      obstacle.pose.position.y - obstacle.scale.y / 2,
                                                                      obstacle.pose.position.z - obstacle.scale.z / 2],
                                                                     [obstacle.pose.position.x + obstacle.scale.x / 2,
                                                                      obstacle.pose.position.y + obstacle.scale.y / 2,
                                                                      obstacle.pose.position.z + obstacle.scale.z / 2],
                                                                     sample_step)
                elif obstacle.type == Marker.SPHERE:
                    self.obstaclesPoints = self.generateSphericalPoints([obstacle.pose.position.x,
                                                                         obstacle.pose.position.y,
                                                                         obstacle.pose.position.z],
                                                                        obstacle.scale.x / 2,
                                                                        sample_step)
            else:
                if obstacle.type == Marker.CUBE:
                    self.obstaclesPoints = np.hstack(
                        (self.obstaclesPoints,
                         self.generateCuboidPoints([obstacle.pose.position.x - obstacle.scale.x / 2,
                                                    obstacle.pose.position.y - obstacle.scale.y / 2,
                                                    obstacle.pose.position.z - obstacle.scale.z / 2],
                                                   [obstacle.pose.position.x + obstacle.scale.x / 2,
                                                    obstacle.pose.position.y + obstacle.scale.y / 2,
                                                    obstacle.pose.position.z + obstacle.scale.z / 2],
                                                   sample_step)))
                elif obstacle.type == Marker.SPHERE:
                    self.obstaclesPoints = np.hstack(
                        (self.obstaclesPoints, self.generateSphericalPoints([obstacle.pose.position.x,
                                                                             obstacle.pose.position.y,
                                                                             obstacle.pose.position.z],
                                                                            obstacle.scale.x / 2,
                                                                            sample_step)))

    def reshapeLocalTraj(self, localTraj):
        if localTraj.shape[0] != 3:
            print("Error: The shape of localTraj is not 3*n")
            return
        elif localTraj.shape[1] != self.localLen:
            print("Error: The length of localTraj is %d, Not equal to localLen" % localTraj.shape[1])
            return

        reshaped = np.zeros((3 * self.localLen, 1))
        for i in range(self.localLen):
            reshaped[(i * 3):(3 * i + 3), 0] = localTraj[:, i]

        return reshaped

    def updateHumanLocalTraj(self, idx, humCmd, curStates):
        force = (humCmd[3] ** 2 + humCmd[4] ** 2 + humCmd[5] ** 2) ** 0.5
        distance = (humCmd[0] ** 2 + humCmd[1] ** 2 + humCmd[2] ** 2) ** 0.5

        # By default, human desired traj is equal to robot desired traj
        # 这里之前发生了一个地址上的copy，对humanLocalTraj的修改同时修改了robotGlobalTraj
        self.humanLocalTraj = self.robotGlobalTraj[:3, idx + 1:(idx + 1 + self.localLen)].copy()

        if distance > 0.01:
            next_state = self.Ad.dot(curStates) + self.Brd.dot(np.zeros((3, 1))) + self.Bhd.dot(
                humCmd[3:])
            forceCmd = humCmd[3:]
            for i in range(self.localLen):
                self.humanLocalTraj[:, i] = next_state[:3].copy().reshape((3,))
                # iterate state space model without robot input to estimate human desired trajectory
                next_state = self.Ad.dot(next_state) + self.Brd.dot(np.zeros((3, 1))) + self.Bhd.dot(forceCmd)

            if force > self.thresholdForce:
                self.humanIntent = 2
            else:
                self.humanIntent = 1
        else:
            self.humanIntent = 0

        # rospy.loginfo("human_%d: (%.2f, %.2f, %.2f)" % (
        #                 idx, curStates[0, 0], curStates[1, 0], curStates[2, 0]))

    def computeLambda(self, curStates):


        # need to optimaize in future works
        if self.obstaclesPoints is None:  # if obstacles is updating, use last lambda value
            return self.lambda_

        # d_res = min(np.linalg.norm(nearestPoint - self.obstaclesPoints, axis=0))

        endEffectorPoint = curStates[0:3].reshape((3, 1))

        # search for closest point in trajectory to current pos (search whole trajectory, also low efficiency)
        # nearestIndex = np.argmin(np.linalg.norm(self.robotGlobalTraj[:3, :] - endEffectorPoint, axis=0))
        # nearestPoint = self.robotGlobalTraj[0:3, nearestIndex].copy().reshape((3, 1))
        # distance = np.linalg.norm(curStates[0:3].reshape((3, 1)) - self.obstaclesPoints, axis=0)
        # obsIdx = np.argmin(distance)
        # obsPoint = self.obstaclesPoints[:, obsIdx].reshape((3, 1))  # nearest obstacle point to current state
        # d_res = np.linalg.norm(nearestPoint - obsPoint)

        # select robot desire point rather than nearest point
        d_res = min(np.linalg.norm(self.robotGlobalTraj[0:3, self.curIdx+1].reshape((3, 1)) - self.obstaclesPoints, axis=0))

        # when obstacles are relative sparse, the value of 'd_res' may be really large 
        # it will lead to overflow error when calculating d_sat
        # so we will set a limits to d_res
        # limits = 0.2
        if d_res > self.deviation:
            d_res = self.deviation

        # nearest version
        # d = np.linalg.norm(endEffectorPoint - nearestPoint)

        # robot desired version
        d = np.linalg.norm(endEffectorPoint - self.robotGlobalTraj[0:3, self.curIdx+1].reshape((3, 1)))

        # normalization --- d_res to 0.1
        norm_k = 0.1 / d_res
        d_norm = d * norm_k

        d_max = min(d_norm, 0.1)
        a1_ = 1  # 在不取max时，d趋向无穷的时候，d_sat趋向于 d_res * a1_
        a2_ = 0.2  # a2_越大，lambda曲线开始时死区越长
        mu_ = 200  # mu_越大，曲线越早达到极限值
        eta_ = 0.1
        d_sat = (a1_ * 0.1) / (1 + eta_ * math.exp(-mu_ * (d_max - a2_ * 0.1))) ** (1 / eta_)

        self.lambda_ = np.sqrt(0.1 ** 2 - d_sat ** 2) / 0.1

        # self.lambda_ = 0.8

        # rospy.loginfo("lambda_: %.2f" % self.lambda_)

        # return self.lambda_

    def pubGlobalTraj(self):
        visualTraj = VisualTraj()
        visualTraj.dimension = len(self.goal)
        visualTraj.trajectory = self.robotGlobalTraj[:len(self.goal),
                                :].T.flatten().tolist()  # pub without velocity msg
        self.vis_pubRawTraj.publish(visualTraj)

    def planGlobalTraj(self, curStates):

        # keep the first-planned trajectory consistent among all users (In User Study)
        if self.loadTraj:
            rospack = rospkg.RosPack()
            self.robotGlobalTraj = np.loadtxt(rospack.get_path('controller') + '/loadTraj/globalTraj.txt').T
            if self.robotGlobalTraj.shape[0] != 6:
                raise Exception("check your globalTraj file, trajectory need to be (6, N), currently is (%d, %d)"
                                % (self.robotGlobalTraj.shape[0], self.robotGlobalTraj.shape[1]))
            self.robotGlobalTrajLen = self.robotGlobalTraj.shape[1]
            self.ori_robotGlobalTrajLen = self.robotGlobalTrajLen
            self.computeGlobalTraj = True

            # visualization
            self.pubGlobalTraj()

            return

        startVel = [0, 0, 0]
        endVel = [0, 0, 0]
        startAcc = [0, 0, 0]
        endAcc = [0, 0, 0]

        # generate trajectory
        rospy.wait_for_service('local_planner')
        try:
            plan_trajectory = rospy.ServiceProxy('local_planner', LocalPlanning)
            trajectoryFlatten = plan_trajectory(curStates.flatten()[:3].tolist(), startVel, startAcc,
                                                self.goal, endVel, endAcc,
                                                -1, -1,
                                                self.time_taken,
                                                self.controlFrequency).trajectory
            self.robotGlobalTraj = np.array(trajectoryFlatten).reshape((-1, 2 * len(self.goal))).T
            if self.robotGlobalTraj.shape[0] != 6:
                raise Exception("Error: The shape of global trajectory is wrong")

            self.robotGlobalTrajLen = self.robotGlobalTraj.shape[1]
            self.ori_robotGlobalTrajLen = self.robotGlobalTrajLen
            self.computeGlobalTraj = True

            # visualization
            self.pubGlobalTraj()

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        # np.savetxt("/home/jun/pHRI_Shared_Control/src/task_publisher/data/fixTraj.txt", self.robotGlobalTraj.T)

    def extendGlobalTraj(self):
        exd_block = np.zeros((2 * len(self.goal), self.replanLen))
        exd_block[0:3, :] = np.array(self.goal).reshape((3, 1))
        self.robotGlobalTraj = np.hstack((self.robotGlobalTraj, exd_block))
        self.robotGlobalTrajLen = self.robotGlobalTraj.shape[1]

    def changeGlobalTraj(self, currentTrajIndex, humCmd):
        startPoint = self.robotGlobalTraj[:3, currentTrajIndex].copy().tolist()
        endPoint = self.robotGlobalTraj[:3, currentTrajIndex + self.replanLen - 1].copy().tolist()
        # startVel = self.robotGlobalTraj[3:6, currentTrajIndex].copy().tolist()
        startVel = np.array([0, 0, 0]).tolist()
        endVel = self.robotGlobalTraj[3:6, currentTrajIndex + self.replanLen - 1].copy().tolist()
        # endVel = np.array([0, 0, 0]).tolist()
        startAcc = np.array([0, 0, 0]).tolist()
        endAcc = np.array([0, 0, 0]).tolist()

        startTime = time.time()
        optimalTraj = None
        minEnergy = np.inf
        i = 0

        # generate feasible trajectories
        while i < self.replanPathNum:
            if time.time() - startTime > self.replanTimeOut:
                rospy.loginfo("Timeout occurs while replanning traj")
                # break
                i = self.replanPathNum

            i += 1
            # generate trajectory
            rospy.wait_for_service('local_planner')
            try:
                plan_trajectory = rospy.ServiceProxy('local_planner', LocalPlanning)
                res = plan_trajectory(startPoint, startVel, startAcc,
                                                    endPoint, endVel, endAcc,
                                                    -1, -1,
                                                    min(self.replanLen, self.ori_robotGlobalTrajLen - currentTrajIndex) * (1 / self.controlFrequency),
                                                    self.controlFrequency)
                if res.success is False:
                    continue
                else:
                    trajectoryFlatten = res.trajectory

                trajectory = np.array(trajectoryFlatten).reshape((-1, 2 * len(startPoint))).T

                # if trajectory.shape != (6, self.replanLen):
                #     raise Exception("Error: The shape of re-planned trajectory is wrong")

            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                continue

            if trajectory.shape != (6, self.replanLen):
                exd_block = np.zeros((2 * len(self.goal), self.replanLen - trajectory.shape[1]))
                exd_block[0:3, :] = np.array(self.goal).reshape((3, 1))
                trajectory = np.hstack((trajectory, exd_block))
            
            if trajectory.shape != (6, self.replanLen):
                raise Exception("Error: The shape of re-planned trajectory is wrong")

            # rospy.loginfo("Generate feasible trajectories")

            originTrajPosition = self.robotGlobalTraj[:3, currentTrajIndex:(currentTrajIndex + self.replanLen)]
            humanForceVector = np.ones((3, self.replanLen))
            humanForceVector[0, :] = humCmd[3]
            humanForceVector[1, :] = humCmd[4]
            humanForceVector[2, :] = humCmd[5]

            # compute energy function
            Ex = 1 / (2 * self.alpha) * trajectory[0, :].dot(self.R.dot(trajectory[0, :].T)) - humanForceVector[0,:].dot(trajectory[0, :].T) - 1 / self.alpha * originTrajPosition[0, :].dot(self.R.dot(trajectory[0, :].T))
            Ey = 1 / (2 * self.alpha) * trajectory[1, :].dot(self.R.dot(trajectory[1, :].T)) - humanForceVector[1,:].dot(trajectory[1, :].T) - 1 / self.alpha * originTrajPosition[1, :].dot(self.R.dot(trajectory[1, :].T))
            Ez = 1 / (2 * self.alpha) * trajectory[2, :].dot(self.R.dot(trajectory[2, :].T)) - humanForceVector[2,:].dot(trajectory[2, :].T) - 1 / self.alpha * originTrajPosition[2, :].dot(self.R.dot(trajectory[2, :].T))

            curEnergy = Ex + Ey + Ez
            # rospy.loginfo("current energy is %.2f" % curEnergy)

            if curEnergy < minEnergy:
                minEnergy = curEnergy
                optimalTraj = trajectory

        # change global trajectory
        if optimalTraj is None:
            rospy.loginfo("update global trajectory failed!")
        else:
            self.robotGlobalTraj[:, currentTrajIndex:(currentTrajIndex + self.replanLen)] = optimalTraj[:, :self.replanLen].copy()
            # visualization
            self.pubGlobalTraj()

        # return self.robotGlobalTraj

    def computeLocalTraj(self, idx, humCmd, curStates):
        if idx + self.localLen > self.robotGlobalTraj.shape[1]:
            raise Exception("Error: Index out of the range of robotGlobalTraj")

        # 当人类有意图时，返回共享控制器计算的局部轨迹
        self.robotLocalTraj = self.robotGlobalTraj[:3, idx + 1:(idx + 1 + self.localLen)]

        X_dr = self.reshapeLocalTraj(self.robotLocalTraj)
        X_dh = self.reshapeLocalTraj(self.humanLocalTraj)
        X_d = np.vstack((X_dh, X_dr))

        # debug
        # self.local_robot_traj = Path()
        # self.local_robot_traj.header.frame_id = self.world_frame
        # pathArray = self.robotLocalTraj.T
        # self.local_robot_traj.poses = []
        # for point in pathArray:
        #     # rospy.loginfo("vis traj append point (%.2f, %.2f, %.2f)" % (point[0], point[1], point[2]))
        #     self.local_robot_traj.poses.append(self.Array2Pose(point))
        # self.vis_pubLocalTraj_r.publish(self.local_robot_traj)

        # self.local_human_traj = Path()
        # self.local_human_traj.header.frame_id = self.world_frame
        # pathArray = self.humanLocalTraj.T
        # self.local_human_traj.poses = []
        # for point in pathArray:
        #     # rospy.loginfo("vis traj append point (%.2f, %.2f, %.2f)" % (point[0], point[1], point[2]))
        #     self.local_human_traj.poses.append(self.Array2Pose(point))
        # self.vis_pubLocalTraj_h.publish(self.local_human_traj)

        # if self.humanIntent != 0:
        #     np.savetxt("/home/jun/pHRI_Shared_Control/src/task_publisher/data/bug/local_r_%d.txt" % (idx), self.robotLocalTraj)
        #     np.savetxt("/home/jun/pHRI_Shared_Control/src/task_publisher/data/bug/local_h_%d.txt" % (idx), self.humanLocalTraj)
        #     np.savetxt("/home/jun/pHRI_Shared_Control/src/task_publisher/data/bug/curStates_%d.txt" % (idx), curStates)
        #     np.savetxt("/home/jun/pHRI_Shared_Control/src/task_publisher/data/bug/lambda_%d.txt" % (idx), np.array([self.lambda_]))
        #     np.savetxt("/home/jun/pHRI_Shared_Control/src/task_publisher/data/bug/humanCmd_%d.txt" % (idx), np.array(humCmd[3:]))

        # rospy.loginfo("curState_%d: (%.2f, %.2f, %.2f)" % (
        #     idx, curStates[0, 0], curStates[1, 0], curStates[2, 0]))

        # 将Q_h和Q_r对角拼接
        Q = np.vstack((np.hstack((self.Qh * self.lambda_, np.zeros((3 * self.localLen, 3 * self.localLen)))),
                       np.hstack((np.zeros((3 * self.localLen, 3 * self.localLen)), self.Qr * (1 - self.lambda_)))))
        # SQ = np.sqrt(Q)
        # SP = np.eye(3 * self.localLen)
        wx = np.vstack((curStates, X_d))

        # s = time.time()

        # tmp1_L_h = np.linalg.pinv(np.vstack((SQ.dot(self.theta_hg), np.sqrt(self.lambda_) * SP)))
        # tmp1_L_h = np.linalg.pinv(np.vstack((SQ.dot(self.theta_hg), np.sqrt(1 - self.lambda_) * SP)))
        # tmp2_L_h = np.vstack((SQ, np.zeros((3 * self.localLen, 6 * self.localLen))))
        # L_h = tmp1_L_h.dot(tmp2_L_h)

        L_h_tmp = np.linalg.inv(np.dot(self.theta_hg.T, Q).dot(self.theta_hg) + (1 - self.lambda_) * np.eye(3 * self.localLen))
        L_h = L_h_tmp.dot(self.theta_hg.T).dot(Q)

        # tmp1_L_r = np.linalg.pinv(np.vstack((SQ.dot(self.theta_rg), np.sqrt(1 - self.lambda_) * SP)))
        # tmp1_L_r = np.linalg.pinv(np.vstack((SQ.dot(self.theta_rg), np.sqrt(self.lambda_) * SP)))
        # tmp2_L_r = np.vstack((SQ, np.zeros((3 * self.localLen, 6 * self.localLen))))
        # L_r = tmp1_L_r.dot(tmp2_L_r)

        L_r_tmp = np.linalg.inv(np.dot(self.theta_rg.T, Q).dot(self.theta_rg) + self.lambda_ * np.eye(3 * self.localLen))
        L_r = L_r_tmp.dot(self.theta_rg.T).dot(Q)

        # e = time.time()
        # print('part5-1-inv:' , e-s)
        # s = time.time()

        H_h = np.hstack((-L_h.dot(self.phi_g), L_h))
        H_r = np.hstack((-L_r.dot(self.phi_g), L_r))

        k_r1 = np.eye(3 * self.localLen) - np.dot(L_r.dot(self.theta_hg), L_h.dot(self.theta_rg))
        k_r2 = H_r - np.dot(L_r.dot(self.theta_hg), H_h)
        k_r = np.linalg.inv(k_r1).dot(k_r2)

        k_h1 = np.eye(3 * self.localLen) - np.dot(L_h.dot(self.theta_rg), L_r.dot(self.theta_hg))
        k_h2 = H_h - np.dot(L_h.dot(self.theta_rg), H_r)
        k_h = np.linalg.inv(k_h1).dot(k_h2)

        # e = time.time()
        # print('part5-2-inv:' , e-s)
        # s = time.time()

        k_0 = np.zeros((3, 3 * self.localLen))
        k_0[:3, :3] = np.eye(3)

        u_r = np.dot(k_0, k_r.dot(wx))
        u_h = np.dot(k_0, k_h.dot(wx))

        # limit max ||u_r|| and ||u_h||
        limit = 12.5
        if np.linalg.norm(u_r) > limit:
            u_r = u_r / np.linalg.norm(u_r) * limit
        if np.linalg.norm(u_h) > limit:
            u_h = u_h / np.linalg.norm(u_h) * limit

        # w_next = self.Ad.dot(curStates) + self.Brd.dot(u_r) + self.Bhd.dot(humCmd[3:])

        if self.humanIntent == 0:
            w_next = self.Ad.dot(curStates) + self.Brd.dot(u_r) + self.Bhd.dot(u_h)
        else:
            w_next = self.Ad.dot(curStates) + self.Brd.dot(u_r) + self.Bhd.dot(humCmd[3:])

        # w_next = self.Ad.dot(curStates) + self.Brd.dot(u_r) + self.Bhd.dot(u_h)
        # cmd_string = str(w_next[0, 0]) + ',' + str(w_next[1, 0]) + ',' + str(w_next[2, 0]) + ',' + str(
        #     w_next[3, 0]) + ',' + str(w_next[4, 0]) + ',' + str(w_next[5, 0])

        # np.set_printoptions(precision = 4)
        # print('-----------')
        # print(curStates.T)
        # print(w_next.T)

        # rospy.loginfo("u_r (%.2f, %.2f, %.2f) u_h (%.2f, %.2f, %.2f)" % (u_r[0], u_r[1], u_r[2], u_h[0], u_h[1], u_h[2]))

        stateCmd = StateCmd()
        stateCmd.stamp = rospy.Time.now()
        stateCmd.x = w_next[0, 0]
        stateCmd.y = w_next[1, 0]
        stateCmd.z = w_next[2, 0]
        stateCmd.vx = w_next[3, 0]
        stateCmd.vy = w_next[4, 0]
        stateCmd.vz = w_next[5, 0]

        return stateCmd

        # return cmd_string

    # generate discrete points on a sphere
    def generateSphericalPoints(self, center, radius, distanceStep):
        sphericalPoints = None
        phi = np.arange(0, np.pi, distanceStep / radius)
        for phi_ in phi:
            radiusCircle = radius * np.sin(phi_)
            if radiusCircle == 0:
                circlePoints_ = np.array([center[0], center[1], radius * np.cos(phi_) + center[2]]).reshape((3, 1))
            else:
                theta = np.arange(0, 2 * np.pi, distanceStep / radiusCircle)
                x = radiusCircle * np.cos(theta) + center[0]
                y = radiusCircle * np.sin(theta) + center[1]
                z = np.ones(len(theta)) * radius * np.cos(phi_) + center[2]
                circlePoints_ = np.vstack((x, y, z))
            if sphericalPoints is None:
                sphericalPoints = circlePoints_
            else:
                sphericalPoints = np.hstack((sphericalPoints, circlePoints_))

        return sphericalPoints

    # generate discrete points on a cubic surface
    def generateCuboidPoints(self, pos1, pos2, distanceStep):
        # 生成长方体的六个面
        x = np.linspace(pos1[0], pos2[0], int(abs(pos2[0] - pos1[0]) / distanceStep))
        y = np.linspace(pos1[1], pos2[1], int(abs(pos2[1] - pos1[1]) / distanceStep))
        z = np.linspace(pos1[2], pos2[2], int(abs(pos2[2] - pos1[2]) / distanceStep))
        # x-y平面
        x1, y1 = np.meshgrid(x, y)
        z1 = np.ones(x1.shape) * pos1[2]
        x2, y2 = np.meshgrid(x, y)
        z2 = np.ones(x2.shape) * pos2[2]
        # x-z平面
        x3, z3 = np.meshgrid(x, z)
        y3 = np.ones(x3.shape) * pos1[1]
        x4, z4 = np.meshgrid(x, z)
        y4 = np.ones(x4.shape) * pos2[1]
        # y-z平面
        y5, z5 = np.meshgrid(y, z)
        x5 = np.ones(y5.shape) * pos1[0]
        y6, z6 = np.meshgrid(y, z)
        x6 = np.ones(y6.shape) * pos2[0]

        # 合并
        cuboidPoints = np.vstack((x1.reshape((1, -1)), y1.reshape((1, -1)), z1.reshape((1, -1))))
        cuboidPoints = np.hstack(
            (cuboidPoints, np.vstack((x2.reshape((1, -1)), y2.reshape((1, -1)), z2.reshape((1, -1))))))
        cuboidPoints = np.hstack(
            (cuboidPoints, np.vstack((x5.reshape((1, -1)), y5.reshape((1, -1)), z5.reshape((1, -1))))))
        cuboidPoints = np.hstack(
            (cuboidPoints, np.vstack((x6.reshape((1, -1)), y6.reshape((1, -1)), z6.reshape((1, -1))))))
        cuboidPoints = np.hstack(
            (cuboidPoints, np.vstack((x3.reshape((1, -1)), y3.reshape((1, -1)), z3.reshape((1, -1))))))
        cuboidPoints = np.hstack(
            (cuboidPoints, np.vstack((x4.reshape((1, -1)), y4.reshape((1, -1)), z4.reshape((1, -1))))))

        return cuboidPoints

    # debug
    # def Array2Pose(self, point):
    #     pose = PoseStamped()
    #     pose.header.frame_id = self.world_frame
    #     pose.header.stamp = rospy.Time.now()
    #     pose.pose.position.x = point[0]
    #     pose.pose.position.y = point[1]
    #     pose.pose.position.z = point[2]
    #     pose.pose.orientation.x = 0
    #     pose.pose.orientation.y = 0
    #     pose.pose.orientation.z = 0
    #     pose.pose.orientation.w = 1
    #     return pose

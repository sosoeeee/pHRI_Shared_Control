#!/usr/bin/env python3
# This file is the implementation of the Minimum jerk/snap trajectory planner

# when using launch file, must set sys.path.append(os.path.dirname(__file__)) to use classes in the same package
import sys
import os

sys.path.append(os.path.dirname(__file__))

from BaseLocalPlanner import BaseLocalPlanner
from PolyTrajectory import PolyTrajectory
import numpy as np
import time

import rospy


class MinimumLocalPlanner(BaseLocalPlanner):
    def __init__(self):
        # private Planner parameters
        self.order = None
        self.opt_order = None
        self.conti_order = None
        self.optimizeT = None
        # result trajectory parameters
        self.polyTrajectories = None
        self.ts = None
        self.durations = None
        self.dim = None

        super(MinimumLocalPlanner, self).__init__()

    def initPlanner(self):
        # get the parameters
        self.order = rospy.get_param("/MinimumLocalPlanner/order", 5)
        self.opt_order = rospy.get_param("/MinimumLocalPlanner/opt_order", 4)
        self.conti_order = rospy.get_param("/MinimumLocalPlanner/conti_order", 3)
        self.optimizeT = rospy.get_param("/MinimumLocalPlanner/optimizeT", True)

    def planTrajectory(self):
        self.dim = len(self.start_vel)
        self.refPath = np.array(self.refPath).reshape((-1, self.dim)).T
        self.polyTrajectories = [PolyTrajectory(self.order) for i in range(self.dim)]

        # if self.refPath.shape[1] > 15:
        #     self.optimizeT = False
        # rospy.loginfo("path length %d is too long, fail to optimize Time params" % self.refPath.shape[1])

        # whether to optimizing time params depends on the std of distance between two next ref 
        # not depends on the number of ref points
        dis = np.linalg.norm(np.diff(self.refPath, axis=1), axis=0)
        std = np.std(dis)
        if std < 0.1:
            self.optimizeT = False

        # whether to optimize the time parameters
        if self.optimizeT and self.refPath.shape[1] > 2:
            self.avr_arrangeTime()  # initialize ts
            self.optimizeTime()
            # update the time parameters
            for i in range(self.dim):
                self.polyTrajectories[i].arrangeTime(self.ts)
        else:
            self.avr_arrangeTime()
            for i in range(self.dim):
                self.polyTrajectories[i].arrangeTime(self.ts)

        for i in range(self.dim):
            coeffs, _ = self.computeSingleAxisTraj(self.refPath[i],
                                                   self.start_vel[i] * self.total_time,
                                                   self.start_acc[i] * self.total_time ** 2,
                                                   self.goal_vel[i] * self.total_time,
                                                   self.goal_acc[i] * self.total_time ** 2)
            self.polyTrajectories[i].setCoeff(coeffs)

        # compute the discrete trajectory (with Time Normalization)
        # traj only contains position and velocity info
        traj = np.zeros((self.dim * 2, int(self.control_frequency * self.total_time)))
        for i in range(int(self.control_frequency * self.total_time)):
            t_ = i / (self.control_frequency * self.total_time)  # Normalized Time t_
            for j in range(self.dim):
                traj[j, i] = self.polyTrajectories[j].getPos(t_)
                traj[j + self.dim, i] = self.polyTrajectories[j].getVel(t_) * (1 / self.total_time)

        return traj

    def optimizeTime(self):
        interation_N = 5
        for n in range(interation_N):

            # rospy.loginfo("start time parameters optimization, iteration is %d" % n)

            # get duration of each segment
            self.durations = np.diff(self.ts)
            cost_T = self.computeCost()

            # t1 = time.time()

            dt = 0.001
            # Calculate Gradient
            grad = np.zeros(len(self.durations))
            for i in range(len(self.durations)):
                d_duration = - np.zeros((len(self.durations)))
                d_duration[i] = dt
                new_durations = self.durations + d_duration
                self.ts = np.hstack((0, np.cumsum(new_durations)))
                cost_T_dDuration = self.computeCost()
                grad[i] = (cost_T_dDuration - cost_T) / dt

            # t2 = time.time()

            # Calculate exact Orientation Gradient
            max_grad = 0
            grad_des_direction = np.zeros(len(self.durations))
            for i in range(len(self.durations)):
                # update self.ts without changing self.total time
                d_duration = - np.ones((len(self.durations))) * (dt / (len(self.durations) - 1))
                d_duration[i] = dt
                new_durations = self.durations + d_duration
                self.ts = np.hstack((0, np.cumsum(new_durations)))
                cost_T_dDuration = self.computeCost()
                # direction_grad = (cost_T_dDuration - cost_T) / np.sum(d_duration ** 2) ** 0.5
                direction_grad = (cost_T_dDuration - cost_T) / dt  # approximation
                if abs(direction_grad) > max_grad:
                    d_duration = d_duration / np.sum(d_duration ** 2) ** 0.5
                    grad_des_direction = d_duration * (-direction_grad / abs(direction_grad))

            # t3 = time.time()

            # backtracking linear search
            step = 0.02
            alpha = 0.2
            beta = 0.5
            while True:
                new_durations = self.durations + grad_des_direction * step
                # ensure duration > 0
                while np.any(new_durations < 0):
                    step /= 2
                    new_durations = self.durations + grad_des_direction * step

                self.ts = np.hstack((0, np.cumsum(new_durations)))
                cost_T_dDuration = self.computeCost()
                if cost_T_dDuration > cost_T + alpha * step * np.dot(grad.reshape((-1, 1)).T,
                                                                     grad_des_direction.reshape((-1, 1))):
                    step *= beta
                else:
                    break

            # t4 = time.time()

            # rospy.loginfo("part1 time takes:" + str(t2 - t1))
            # rospy.loginfo("part2 time takes:" + str(t3 - t2))
            # rospy.loginfo("part3 time takes:" + str(t4 - t3))

            # update time parameters
            self.durations = self.durations + grad_des_direction * step
            self.ts = np.hstack((0, np.cumsum(self.durations)))

    def avr_arrangeTime(self):
        dx = np.diff(self.refPath)

        distance = np.sum(np.sqrt(np.sum(dx ** 2, axis=0)))
        arvSpeed = distance / 1  # Time Normalization

        self.ts = [0]
        for i in range(self.refPath.shape[1] - 1):
            self.ts.append(self.ts[i] + np.sqrt(np.sum(dx[:, i] ** 2)) / arvSpeed)

        self.ts = np.array(self.ts)

    def computeQ(self, n, r, t1, t2):
        # n:polynormial order
        # r:derivertive order, 1:minimum vel 2:minimum acc 3:minimum jerk 4:minimum snap
        # t1:start timestamp for polynormial
        # t2:end timestap for polynormial

        T = np.zeros(((n - r) * 2 + 1, 1))
        for i in range((n - r) * 2 + 1):
            T[i] = t2 ** (i + 1) - t1 ** (i + 1)

        Q = np.zeros((n + 1, n + 1))
        for i in range(r + 1, n + 2):  # i:row, 从1开始
            for j in range(i, n + 2):
                k1 = i - r - 1
                k2 = j - r - 1
                k = k1 + k2 + 1
                Q[i - 1, j - 1] = np.prod(np.arange(k1 + 1, k1 + r + 1)) * np.prod(np.arange(k2 + 1, k2 + r + 1)) / k * \
                                  T[k - 1]
                Q[j - 1, i - 1] = Q[i - 1, j - 1]

        return Q

    def computeCost(self):
        cost_all = 0
        for i in range(self.dim):
            _, cost = self.computeSingleAxisTraj(self.refPath[i],
                                                 self.start_vel[i] * self.total_time,
                                                 self.start_acc[i] * self.total_time ** 2,
                                                 self.goal_vel[i] * self.total_time,
                                                 self.goal_acc[i] * self.total_time ** 2)
            cost_all += cost
        return cost_all

    def computeSingleAxisTraj(self, path, v0, a0, vt, at):
        path = path.reshape((1, -1))
        n_coef = self.order + 1  # the number of coefficient
        n_seg = path.shape[1] - 1  # the number of segments

        # compute Q
        Q = np.zeros((n_coef * n_seg, n_coef * n_seg))
        for k in range(n_seg):
            Q_k = self.computeQ(self.order, self.opt_order, self.ts[k], self.ts[k + 1])
            Q[k * n_coef:(k + 1) * n_coef, k * n_coef:(k + 1) * n_coef] = Q_k

        # compute Tk Tk(i,j) = ts(i)^(j)
        Tk = np.zeros((n_seg + 1, n_coef))
        for i in range(n_coef):
            Tk[:, i] = self.ts ** i

        # compute A
        n_continuous = self.conti_order  # 1:p  2:pv  3:pva  4:pvaj  5:pvajs

        if n_continuous * 2 != n_coef:
            raise Exception("Try to set poly order to %d in order to satisfy the continuous constrain" % (n_coef - 1))

        A = np.zeros((n_continuous * 2 * n_seg, n_coef * n_seg))
        for i in range(1, n_seg + 1):
            for j in range(1, n_continuous + 1):
                for k in range(j, n_coef + 1):
                    # if k == j:
                    #     t1 = 1
                    #     t2 = 1
                    # else:
                    #     t1 = Tk[i - 1, k - j]
                    #     t2 = Tk[i, k - j]
                    t1 = Tk[i - 1, k - j]
                    t2 = Tk[i, k - j]
                    A[n_continuous * 2 * (i - 1) + j - 1, n_coef * (i - 1) + k - 1] = np.prod(
                        np.arange(k - j + 1, k)) * t1
                    A[n_continuous * 2 * (i - 1) + j - 1 + n_continuous, n_coef * (i - 1) + k - 1] = np.prod(
                        np.arange(k - j + 1, k)) * t2

        # compute M
        M = np.zeros((n_continuous * 2 * n_seg, n_continuous * (n_seg + 1)))
        for i in range(1, 2 * n_seg + 1):
            j = int(np.floor(i / 2)) + 1
            rbeg = (i - 1) * n_continuous + 1
            cbeg = (j - 1) * n_continuous + 1
            M[rbeg - 1:rbeg + n_continuous - 1, cbeg - 1:cbeg + n_continuous - 1] = np.eye(n_continuous)

        # compute C
        num_d = n_continuous * (n_seg + 1)
        C = np.eye(num_d)
        df = np.hstack((path, np.array(v0).reshape((1, -1)), np.array(a0).reshape((1, -1)), np.array(vt).reshape((1, -1)),
                        np.array(at).reshape((1, -1)))).T
        fix_idx = np.array(range(1, num_d, n_continuous))
        # add vel and acc constrain at start and end of Traj
        fix_idx = np.hstack((fix_idx, 2, 3, num_d - n_continuous + 2, num_d - n_continuous + 3))
        free_idx = np.setdiff1d(range(1, num_d + 1), fix_idx)
        C = np.hstack((C[:, fix_idx - 1], C[:, free_idx - 1]))

        AiMC = np.linalg.inv(A).dot(M).dot(C)
        R = AiMC.T.dot(Q).dot(AiMC)

        n_fix = len(fix_idx)
        # Rff = R[:n_fix, :n_fix]
        Rfp = R[:n_fix, n_fix:]
        # Rpf = R[n_fix:, :n_fix]
        Rpp = R[n_fix:, n_fix:]

        dp = -np.linalg.inv(Rpp).dot(Rfp.T).dot(df)

        p = AiMC.dot(np.vstack((df, dp)))

        ploys = p.reshape((n_coef, n_seg), order='F')

        cost = p.T.dot(Q).dot(p)[0, 0]

        return ploys, cost

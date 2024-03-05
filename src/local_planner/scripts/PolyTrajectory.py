#!/usr/bin/env python3
# This file is the implementation of the class of trajectory parameterized by piecewise-polynomials curve
import numpy as np
import math


class PolyTrajectory:
    def __init__(self, order=5):
        self.order = order
        self.totalT = 0
        self.T_set = [0]  # exact time at each segment point, contains start and end time
        self.coeff = None

    def getTotalTime(self):
        return self.totalT

    def arrangeTime(self, t_set):
        self.T_set = t_set
        self.totalT = self.T_set[-1]

    def setCoeff(self, coeff):
        # check whether having arranged the Time
        if len(self.T_set) == 1:
            raise Exception("You need parameterize Trajectory Time Parameters first")

        elif coeff.shape == (self.order, len(self.T_set) - 1):
            raise Exception("Coefficient shape is wrong, expected (%d, %d)" % (self.order, len(self.T_set) - 1))

        self.coeff = coeff

    # get the index of corresponding Ploy-nominal at time "t"
    def getIndex(self, t):
        if t > self.totalT:
            raise Exception("t can't exceed the total duration of Trajectory %d " % self.totalT)

        # Dichotomy
        upper = len(self.T_set)
        lower = 0
        while upper - lower > 1:
            idx = int((lower + upper) / 2)
            if t >= self.T_set[idx]:
                lower = idx
            else:
                upper = idx

        # idx_ = np.where(self.Ts<=t_)[0]

        return lower

    def getPos(self, t):
        idx = self.getIndex(t)
        res = self.coeff[0, idx]
        for i in range(1, self.order + 1):
            res += self.coeff[i, idx] * t ** i

        return res

    def getVel(self, t):
        idx = self.getIndex(t)
        res = self.coeff[1, idx]
        for i in range(2, self.order + 1):
            res += self.coeff[i, idx] * i * t ** (i - 1)

        return res

    def getValue(self, t, diff):
        # diff 轨迹导数的阶数
        if diff > self.order:
            return 0
        else:
            idx = self.getIndex(t)
            res = self.coeff[diff, idx] * math.factorial(diff)
            for i in range(diff+1, self.order+1):
                res += self.coeff[i, idx] * (math.factorial(i)/math.factorial(i - diff)) * t ** (i - diff)

            return res

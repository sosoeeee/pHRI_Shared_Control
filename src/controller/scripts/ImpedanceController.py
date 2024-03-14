#!/usr/bin/env python3
import sys
import os

# Add the current directory to the path module, in order to import the LocalPlannerClass
sys.path.append(os.path.dirname(__file__))

import rospy
from BaseController import BaseController

import numpy as np


class ImpedanceController(BaseController):
    def __init__(self):
        super(ImpedanceController, self).__init__()

        # controller parameters
        # Impedance Model
        self.Md = None
        self.Cd = None
        # discrete state space model
        self.Ad = None
        self.Brd = None
        self.Bhd = None

        self.loadParams()

    def loadParams(self):
        # read parameters
        self.Md = rospy.get_param("/impedance_controller/M", 1)
        self.Cd = rospy.get_param("/impedance_controller/C", 1)

        # state space modelling
        Md_inv = np.linalg.inv(self.Md * np.eye(3))
        A = np.zeros((6, 6))
        A[0:3, 3:6] = np.eye(3)
        A[3:6, 3:6] = -Md_inv.dot(self.Cd * np.eye(3))
        Br = np.zeros((6, 3))
        Br[3:6, :] = Md_inv
        Bh = np.zeros((6, 3))
        Bh[3:6, :] = Md_inv
        C = np.zeros((3, 6))
        C[:, 0:3] = np.eye(3)

        # discretization
        T = 1 / self.controlFrequency
        self.Ad = np.eye(6) + A * T
        self.Brd = Br * T
        self.Bhd = Bh * T

    def computeCmd(self):
        nextState = self.Ad.dot(self.currentStates) + self.Brd.dot(np.zeros((3, 1))) + self.Bhd.dot(self.humanCmd[3:])
        cmd_string = str(nextState[0, 0]) + ',' + str(nextState[1, 0]) + ',' + str(nextState[2, 0]) + ',' + str(nextState[3, 0]) + ',' + str(nextState[4, 0]) + ',' + str(nextState[5, 0])

        return cmd_string


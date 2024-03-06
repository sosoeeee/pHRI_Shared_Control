#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from actuator.srv import *

from abc import abstractmethod
import numpy as np
import time


class BaseController:
    def __init__(self):
        # subscribe human command
        rospy.Subscriber("stickSignal", String, self.humanCmd_callback)
        self.humanCmd = np.zeros((6, 1))

        # subscribe robot states from actuator
        rospy.Subscriber('/actuator/robotState', PoseStamped, self.cartesianState_callBack, queue_size=1)
        self.robotReady = False
        self.currentStates = np.zeros((6, 1))  # pos (x, y, z) and vel (dx, dy, dz) in Cartesian Space(task space)
        # variables used to compute velocity by position differentiating
        self.firstSubFlag = True
        self.startTime = 0
        self.endTime = 0
        self.lastPos = np.zeros((3, 1))

        # publish control command to robot
        self.pubControlCmd = rospy.Publisher('nextState', String, queue_size=10)

        # private variables for controlling
        self.controlFrequency = rospy.get_param("/controller/control_frequency", 10)

        # initialize parameters in a specific controller
        self.initController()

    def humanCmd_callback(self, msg):
        posAndForce = msg.data.split(',')
        # haptic stick pos (x, y, z)
        self.humanCmd[0] = float(posAndForce[0])
        self.humanCmd[1] = float(posAndForce[1])
        self.humanCmd[2] = float(posAndForce[2])
        # interactive force (x, y, z) between human and remote haptic device
        self.humanCmd[3] = float(posAndForce[3])
        self.humanCmd[4] = float(posAndForce[4])
        self.humanCmd[5] = float(posAndForce[5])

    def cartesianState_callBack(self, msg):
        self.currentStates[0] = msg.pose.position.x
        self.currentStates[1] = msg.pose.position.y
        self.currentStates[2] = msg.pose.position.z

        if self.firstSubFlag:
            self.lastPos = self.currentStates[:3]
            self.startTime = time.time()
            self.firstSubFlag = False
        else:
            self.endTime = time.time()
            deltaT = self.endTime - self.startTime
            self.startTime = time.time()
            # update vel
            self.currentStates[3] = (self.currentStates[0] - self.lastPos[0]) / deltaT
            self.currentStates[4] = (self.currentStates[1] - self.lastPos[1]) / deltaT
            self.currentStates[5] = (self.currentStates[2] - self.lastPos[2]) / deltaT
            if self.currentStates[3] < 0.0001:
                self.currentStates[3] = 0
            if self.currentStates[4] < 0.0001:
                self.currentStates[4] = 0
            if self.currentStates[5] < 0.0001:
                self.currentStates[5] = 0
            self.lastPos = self.currentStates[:3]

    def getCurrentState(self):
        return self.currentStates

    def getHumanCmd(self):
        return self.humanCmd

    def waitRobotReady(self):
        rospy.wait_for_service('isRobotReady')
        while True:
            try:
                check = rospy.ServiceProxy('isRobotReady', isRobotReady)
                self.robotReady = check().isReady
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)

            if self.robotReady:
                break
            else:
                rospy.sleep(0.1)

    @abstractmethod
    def initController(self):
        pass

    @abstractmethod
    def run(self):
        pass

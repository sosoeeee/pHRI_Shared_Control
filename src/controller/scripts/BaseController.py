#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from task_publisher.msg import ReachGoal
from actuator.srv import *
from actuator.msg import StateVector
from controller.msg import StateCmd

from abc import abstractmethod
import numpy as np
import threading

Lock = threading.Lock()


class BaseController:
    def __init__(self):
        # subscribe human command
        rospy.Subscriber("stickSignal", String, self.humanCmd_callback, queue_size=1)
        self.twoDimFlag = rospy.get_param("/two_dimesion", False)
        self.humanCmd = np.zeros((6, 1))
        self.rawBuffer = np.zeros((6, 3))
        self.filterBuffer = np.zeros((6, 3))
        # Cutoff Frequency is 10Hz (in 100Hz sample freq)
        self.filter_b = [0.0675, 0.1349, 0.0675]
        self.filter_a = [1, -1.1430, 0.4128]

        # subscribe robot states from actuator
        rospy.Subscriber('/actuator/robotState', StateVector, self.cartesianState_callBack, queue_size=10)
        self.robotReady = False
        self.currentStates = np.zeros((6, 1))  # pos (x, y, z) and vel (dx, dy, dz) in Cartesian Space(task space)

        # publish control command to robot
        self.controlFrequency = rospy.get_param("/controller/control_frequency", 10)
        self.pubControlCmd = rospy.Publisher('nextState', StateCmd, queue_size=1)
        # self.pubCmdTimer = rospy.Timer(rospy.Duration(1/self.controlFrequency), self.pubCmd_callback)

        # subscribe task publisher (currently care about "ReachGoal" task only)
        rospy.Subscriber('/task/reachGoal', ReachGoal, self.task_callBack, queue_size=1)
        self.active = False  # only becomes True while task executing
        self.goal = None
        self.time_taken = None
        self.tolerance = None

        # wait actuator ready
        self.waitRobotReady()

    def humanCmd_callback(self, msg):
        posAndForce = msg.data.split(',')
        # The sampling frequency of haptic device signal is 100Hz
        # update raw buffer
        self.rawBuffer[:, 2] = self.rawBuffer[:, 1]
        self.rawBuffer[:, 1] = self.rawBuffer[:, 0]
        
        # two dimension setup
        if self.twoDimFlag:
            self.rawBuffer[:, 0] = np.array([float(posAndForce[0]), float(posAndForce[1]), float(posAndForce[2]),
                                            -float(posAndForce[3]), -float(posAndForce[4]), 0])
        else:
            self.rawBuffer[:, 0] = np.array([float(posAndForce[0]), float(posAndForce[1]), float(posAndForce[2]),
                                         -float(posAndForce[3]), float(posAndForce[5]), -float(posAndForce[4])])

        # update filter buffer
        self.filterBuffer[:, 2] = self.filterBuffer[:, 1]
        self.filterBuffer[:, 1] = self.filterBuffer[:, 0]
        self.filterBuffer[:, 0] = (self.filter_b[0] * self.rawBuffer[:, 0] +
                                   self.filter_b[1] * self.rawBuffer[:, 1] +
                                   self.filter_b[2] * self.rawBuffer[:, 2] -
                                   self.filter_a[1] * self.filterBuffer[:, 1] -
                                   self.filter_a[2] * self.filterBuffer[:, 2]) / self.filter_a[0]

        # filter data
        self.humanCmd = np.array([data if abs(data) > 0.001 else 0 for data in self.filterBuffer[:, 0]]).reshape((6, 1))

        # posAndForce = msg.data.split(',')
        # # haptic stick pos (x, y, z)
        # self.humanCmd[0] = float(posAndForce[0])
        # self.humanCmd[1] = float(posAndForce[1])
        # self.humanCmd[2] = float(posAndForce[2])
        # # interactive force (x, y, z) between human and remote haptic device
        # self.humanCmd[3] = -float(posAndForce[3])
        # self.humanCmd[4] = float(posAndForce[5])
        # self.humanCmd[5] = -float(posAndForce[4])

    def cartesianState_callBack(self, msg):
        self.currentStates[0] = msg.x
        self.currentStates[1] = msg.y
        self.currentStates[2] = msg.z
        self.currentStates[3] = msg.dx
        self.currentStates[4] = msg.dy
        self.currentStates[5] = msg.dz

    def task_callBack(self, msg):
        self.active = True
        self.goal = [msg.goal[0], msg.goal[1], msg.goal[2]]
        self.time_taken = msg.time_taken
        self.tolerance = msg.tolerance
        # rospy.loginfo("Controller is activated !")

    # unstable
    # def pubCmd_callback(self, event):
    #     if self.robotReady and self.active:
    #         cmd = self.computeCmd()
    #         self.pubControlCmd.publish(cmd)

    #         # rospy.loginfo('cmd' + cmd)

    #         # deactivate the controller when task is completed
    #         if np.linalg.norm(self.currentStates[:3] - np.array(self.goal)) < self.tolerance:
    #             self.active = False
    #             self.reInitial()
    #             rospy.loginfo("Task has completed, controller is deactivated !")
            
    #         rospy.loginfo("real control frequency is %.2f" % (1/event.last_duration))
    #     else:
    #         return
        
    def run(self):
        rate = rospy.Rate(self.controlFrequency)
        while not rospy.is_shutdown():
            if self.robotReady and self.active:
                
                with Lock:
                    cmd = self.computeCmd()
                    self.pubControlCmd.publish(cmd)

                # rospy.loginfo('cmd' + cmd)

                # deactivate the controller when task is completed
                if np.linalg.norm(self.currentStates[:3] - np.array(self.goal).reshape((3, 1))) < self.tolerance:
                    self.active = False
                    self.reInitial()
                    rospy.loginfo("Task has completed, controller is deactivated !")
            else:
                pass
            
            rate.sleep()

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
                rospy.loginfo("Actuator finish initialization, Robot is ready!")
                break
            else:
                rospy.sleep(0.1)

    @abstractmethod
    def loadParams(self):
        pass

    @abstractmethod
    def computeCmd(self):
        pass

    @abstractmethod
    def reInitial(self):
        pass

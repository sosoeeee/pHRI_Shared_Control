#! /usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

import numpy as np
from abc import abstractmethod


# used to subscribe system(human and robots) info for task executing
class BaseTaskServer(object):
    def __init__(self):
        # subscribe human force
        rospy.Subscriber("stickSignal", String, self.humanCmd_callback, queue_size=1)
        self.humanForce = np.zeros((3, 1))

        # subscribe robot states from actuator
        rospy.Subscriber('/actuator/robotState', PoseStamped, self.cartesianState_callBack, queue_size=1)
        self.currentStates = np.zeros((3, 1))  # pos (x, y, z) in Cartesian Space(task space)

    def humanCmd_callback(self, msg):
        posAndForce = msg.data.split(',')
        # interactive force (x, y, z) between human and remote haptic device
        self.humanForce[0] = float(posAndForce[3])
        self.humanForce[1] = float(posAndForce[4])
        self.humanForce[2] = float(posAndForce[5])

    def cartesianState_callBack(self, msg):
        self.currentStates[0] = msg.pose.position.x
        self.currentStates[1] = msg.pose.position.y
        self.currentStates[2] = msg.pose.position.z

    @abstractmethod
    def execute_cb(self, goal):
        pass

    @abstractmethod
    def updateInterface(self):
        pass

    @abstractmethod
    def recordData(self):
        pass

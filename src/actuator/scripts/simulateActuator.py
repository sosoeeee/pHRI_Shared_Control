#!/usr/bin/env python3
import sys
import os

# Add the current directory to the path module, in order to import the LocalPlannerClass
sys.path.append(os.path.dirname(__file__))


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PointStamped
from actuator.srv import isRobotReady, isRobotReadyResponse
from actuator.msg import StateVector


import numpy as np


# No dynamics simulation
class simActuator:
    def __init__(self):
        # publish cmd to Rviz
        self.pubPosCmd = rospy.Publisher('/simulate_actuator/pos', PointStamped, queue_size=10)

        # publish simulate point state to controller
        self.pubRobotState = rospy.Publisher('/actuator/robotState', StateVector, queue_size=10)
        self.pubStateFreq = rospy.get_param("/simulate_actuator/pub_frequency", 100)
        self.pubStateTimer = rospy.Timer(rospy.Duration(1/self.pubStateFreq), self.pubStateTimer_callback)

        # subscribe controller cmd
        rospy.Subscriber('/nextState', String, self.controlCmd_callback, queue_size=1)

        # simulate point variable
        self.pos = np.zeros((3, 1))
        self.vel = np.zeros((3, 1))

        # private variables
        self.world_frame = rospy.get_param('/world_frame', 'map')

        # add service to check whether robot has initialized
        self.isReady_service = rospy.Service('isRobotReady', isRobotReady, self.handle_isRobotReady)

        self.readyFlag = False
        self.initPos = rospy.get_param("/simulate_actuator/initPos", None)

    def initRobot(self):
        if self.initPos is None:
            raise Exception("fail to get initial position of simulated point")
        else:
            self.pos[0] = self.initPos['x']
            self.pos[1] = self.initPos['y']
            self.pos[2] = self.initPos['z']
            rospy.sleep(2)
            self.readyFlag = True

    def handle_isRobotReady(self, req):
        # return the response
        res = isRobotReadyResponse()
        res.isReady = self.readyFlag
        return res

    def controlCmd_callback(self, msg):
        cmd = msg.data
        [x, y, z, vx, vy, vz] = cmd.split(',')
        x = float(x)
        y = float(y)
        z = float(z)
        vx = float(vx)
        vy = float(vy)
        vz = float(vz)

        self.pos[0] = x
        self.pos[1] = y
        self.pos[2] = z
        self.vel[0] = vx
        self.vel[1] = vy
        self.vel[2] = vz

        pointCmd = PointStamped()
        pointCmd.point.x = self.pos[0]
        pointCmd.point.y = self.pos[1]
        pointCmd.point.z = self.pos[2]
        pointCmd.header.frame_id = self.world_frame

        self.pubPosCmd.publish(pointCmd)

    def pubStateTimer_callback(self, event):
        stateVector = StateVector()
        stateVector.x = self.pos[0]
        stateVector.y = self.pos[1]
        stateVector.z = self.pos[2]
        stateVector.dx = self.vel[0]
        stateVector.dy = self.vel[1]
        stateVector.dz = self.vel[2]

        self.pubRobotState.publish(stateVector)

    def run(self):
        self.initRobot()
        rospy.spin()

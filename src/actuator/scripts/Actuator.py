#!/usr/bin/env python3
import sys
import os

# Add the current directory to the path module, in order to import the LocalPlannerClass
sys.path.append(os.path.dirname(__file__))


import rospy
from iiwa_msgs.msg import JointPosition
from iiwa_msgs.msg import CartesianPose
from iiwa_msgs.msg import JointVelocity
from iiwa_msgs.msg import JointPositionVelocity
from std_msgs.msg import String
from actuator.srv import isRobotReady, isRobotReadyResponse, initRobot, initRobotResponse
from actuator.msg import StateVector


import numpy as np
import time
from Jacobian import Jacobian


class Actuator:
    def __init__(self):
        # publish cmd to real robot
        self.pubPoseCmd = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=10)
        self.pubVelCmd = rospy.Publisher('/iiwa/command/JointVelocity', JointVelocity, queue_size=10)

        # publish robot state to controller
        self.pubRobotState = rospy.Publisher('/actuator/robotState', StateVector, queue_size=10)

        # subscribe controller cmd
        rospy.Subscriber('/nextState', String, self.controlCmd_callback, queue_size=1)
        self.firstSubFlag = True
        self.startTime = None
        self.endTime = None
        self.lastPos = None

        # subscribe joint states
        rospy.Subscriber('/iiwa/state/JointPositionVelocity', JointPositionVelocity, self.jointState_callback, queue_size=1)
        rospy.Subscriber('/iiwa/state/CartesianPose', CartesianPose, self.cartesianState_callBack, queue_size=1)

        # private variables
        self.control_frequency = rospy.get_param("/controller/control_frequency", 10)
        self.currentJointPosition = None
        self.currentJointVelocity = None

        # add service to check whether robot has initialized
        self.isReady_service = rospy.Service('isRobotReady', isRobotReady, self.handle_isRobotReady)
        self.initRobot_service = rospy.Service('initRobot', initRobot, self.handle_initRobot)

        self.readyFlag = False
        self.initJointPose = rospy.get_param("/actuator/initJointPos", None)

    def initRobot(self):
        while self.currentJointPosition is None:
            rospy.loginfo("Can't receive robot state info, please check your connection")
            rospy.sleep(0.5)

        if self.initJointPose is None:
            raise Exception("fail to get initial position of robot joints")
        else:
            cmdJointPosition = JointPosition()
            cmdJointPosition.position.a1 = self.initJointPose['q1']
            cmdJointPosition.position.a2 = self.initJointPose['q2']
            cmdJointPosition.position.a3 = self.initJointPose['q3']
            cmdJointPosition.position.a4 = self.initJointPose['q4']
            cmdJointPosition.position.a5 = self.initJointPose['q5']
            cmdJointPosition.position.a6 = self.initJointPose['q6']
            cmdJointPosition.position.a7 = self.initJointPose['q7']
            self.pubPoseCmd.publish(cmdJointPosition)
            
            while abs(self.currentJointPosition[0] - self.initJointPose['q1']) > 0.01 or \
                  abs(self.currentJointPosition[1] - self.initJointPose['q2']) > 0.01 or \
                  abs(self.currentJointPosition[2] - self.initJointPose['q3']) > 0.01 or \
                  abs(self.currentJointPosition[3] - self.initJointPose['q4']) > 0.01 or \
                  abs(self.currentJointPosition[4] - self.initJointPose['q5']) > 0.01 or \
                  abs(self.currentJointPosition[5] - self.initJointPose['q6']) > 0.01 or \
                  abs(self.currentJointPosition[6] - self.initJointPose['q7']) > 0.01:
                rospy.sleep(0.5)

            self.readyFlag = True

    def handle_isRobotReady(self, req):
        # return the response
        res = isRobotReadyResponse()
        res.isReady = self.readyFlag
        return res

    def handle_initRobot(self, req):
        self.initRobot()
        return True

    def controlCmd_callback(self, msg):
        cmd = msg.data
        [x, y, z, vx, vy, vz] = cmd.split(',')
        x = float(x)
        y = float(y)
        z = float(z)
        vx = float(vx)
        vy = float(vy)
        vz = float(vz)

        Jcob = Jacobian(self.currentJointPosition)
        currentJointPositionVector = np.array(self.currentJointPosition).reshape((7, 1))
        Jcob = Jcob[:3, :]
        # 求解伪逆
        JcobInv = np.linalg.pinv(Jcob)
        # 计算关节角速度
        qdot = JcobInv.dot(np.array([vx, vy, vz]).reshape((3, 1)))
        # 计算关节角度
        q = currentJointPositionVector + qdot / self.control_frequency
        print("q", q)
        # # 发布关节角度指令
        cmdJointPosition = JointPosition()
        cmdJointPosition.position.a1 = q[0]
        cmdJointPosition.position.a2 = q[1]
        cmdJointPosition.position.a3 = q[2]
        cmdJointPosition.position.a4 = q[3]
        cmdJointPosition.position.a5 = q[4]
        cmdJointPosition.position.a6 = q[5]
        cmdJointPosition.position.a7 = q[6]

        self.pubPoseCmd.publish(cmdJointPosition)

    def jointState_callback(self, msg):
        self.currentJointPosition = self.toArray(msg.position)
        self.currentJointVelocity = self.toArray(msg.velocity)

    def cartesianState_callBack(self, msg):
        stateVector = StateVector()
        stateVector.x = msg.pose.position.x
        stateVector.y = msg.pose.position.y
        stateVector.z = msg.pose.position.z

        if self.firstSubFlag:
            self.lastPos = np.array([stateVector.x, stateVector.y, stateVector.z])
            self.startTime = time.time()
            self.firstSubFlag = False
        else:
            self.endTime = time.time()
            deltaT = self.endTime - self.startTime
            self.startTime = time.time()
            # update vel
            stateVector.dx = (msg.pose.position.x - self.lastPos[0]) / deltaT
            stateVector.dy = (msg.pose.position.y - self.lastPos[1]) / deltaT
            stateVector.dz = (msg.pose.position.z - self.lastPos[2]) / deltaT
            if stateVector.dx < 0.0001:
                stateVector.dx = 0
            if stateVector.dy < 0.0001:
                stateVector.dy = 0
            if stateVector.dz < 0.0001:
                stateVector.dz = 0
            self.lastPos = np.array([stateVector.x, stateVector.y, stateVector.z])

        self.pubRobotState.publish(stateVector)

    def run(self):
        self.initRobot()
        rospy.spin()

    # 将JointPositionVelocity消息转换为numpy数组
    def toArray(self, msg):
        ans = np.zeros(7)
        ans[0] = msg.a1
        ans[1] = msg.a2
        ans[2] = msg.a3
        ans[3] = msg.a4
        ans[4] = msg.a5
        ans[5] = msg.a6
        ans[6] = msg.a7
        return ans
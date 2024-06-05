#!/usr/bin/env python
import rospy
from std_msgs.msg import String

# simulating force signal(100Hz)
Max = 4
freq = 100
speed = 0.3
step = speed/freq
currentForce = 0
signal = 1
mode = 1 # 1: act 0: no act
noActTime = 2
ctr = 0


def force_pub():
     global currentForce, signal, mode, ctr
     pub = rospy.Publisher('stickSignal', String, queue_size=1)
     rospy.init_node('force_simulator', anonymous=True)
     rate = rospy.Rate(freq)
     while not rospy.is_shutdown():
          if mode == 1:
               currentForce += signal * step
               # print(currentForce)
               if currentForce > Max or currentForce < -Max:
                    signal *= -1
               data = "0.02,0,0," + str(currentForce) + ",0,0"
          elif mode == 0:
               ctr += 1
               data = "0,0,0,0,0,0"
               
          if abs(currentForce) < 0.01 and signal*currentForce < 0:
               mode = 0
               currentForce = 0
          if ctr > noActTime * freq:
               mode = 1
               ctr = 0

          pub.publish(data)
          rate.sleep()

if __name__ == '__main__':
     try:
          force_pub()
     except rospy.ROSInterruptException:
          pass
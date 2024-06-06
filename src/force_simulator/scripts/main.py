#!/usr/bin/env python
import rospy
from std_msgs.msg import String

# simulating force signal(100Hz)
Max = 12
freq = 100
speed = 0.2
step = speed/freq
currentForce = 0
signal = 1
noActTime = 3
ctr = 0

# square signal
step_square = 0.5
lastTime = 20
ctr_square = 0

mode = 3 # 1: triangle act 0: no act 2: square act

def force_pub():
     global currentForce, signal, mode, ctr, ctr_square
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
          elif mode == 3:
               if ctr_square > lastTime * freq:
                    ctr_square = 0
                    currentForce += step_square
               if currentForce > Max:
                    currentForce = 0
               ctr_square += 1

               if currentForce > 0:
                    data = "0.02,0,0," + str(currentForce) + ",0,0"
               else:
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
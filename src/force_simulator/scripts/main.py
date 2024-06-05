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

def force_pub():
     global currentForce, signal
     pub = rospy.Publisher('stickSignal', String, queue_size=1)
     rospy.init_node('force_simulator', anonymous=True)
     rate = rospy.Rate(freq)
     while not rospy.is_shutdown():
          currentForce += signal * step
          # print(currentForce)
          if currentForce > Max or currentForce < -Max:
               signal *= -1

          data = "0.02,0,0," + str(currentForce) + ",0.05,0"
          pub.publish(data)
          rate.sleep()

if __name__ == '__main__':
     try:
          force_pub()
     except rospy.ROSInterruptException:
          pass
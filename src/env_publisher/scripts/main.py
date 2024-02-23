#!/usr/bin/env python3
import sys
import os

sys.path.append(os.path.dirname(__file__)) # when using launch file, must be global path :( so directly use "os.path.dirname(__file__)"

from EnvPublisherClass import EnvPublisher
import rospy

if __name__ == '__main__':
    try:
        rospy.init_node('env_publisher', anonymous=False)
        envPublisher = EnvPublisher()
        envPublisher.run()

    except rospy.ROSInterruptException:
        pass

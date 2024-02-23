#!/usr/bin/env python3
import sys

sys.path.append("src/env_publisher/scripts") # the path relative to ROS workspace

from EnvPublisherClass import EnvPublisher
import rospy

if __name__ == '__main__':
    try:
        rospy.init_node('env_publisher', anonymous=False)
        envPublisher = EnvPublisher()
        envPublisher.run()

    except rospy.ROSInterruptException:
        pass

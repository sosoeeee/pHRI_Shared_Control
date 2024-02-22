#!/usr/bin/env python3

from EnvPublisher import *

if __name__ == '__main__':
    try:
        rospy.init_node('env_publisher', anonymous=False)
        envPublisher = EnvPublisher()
        envPublisher.run()

    except rospy.ROSInterruptException:
        pass

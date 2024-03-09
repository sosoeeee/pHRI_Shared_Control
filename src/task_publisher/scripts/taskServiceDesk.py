#!/usr/bin/env python3
import sys
import os

# Add the current directory to the path module, in order to import the LocalPlannerClass
sys.path.append(os.path.dirname(__file__))

import rospy
from taskClients import *


# read task from yaml file, and instantiate them all
if __name__ == '__main__':
    try:
        rospy.init_node('task_service_desk', anonymous=False)

        taskList = rospy.get_param("/task_list", [])
        clientList = []

        # initial clients
        for task in taskList:
            if task['task_type'] == 'reachGoal':
                clientList.append(PubGoalActionClient(task))

        for client in clientList:
            client.sendReq()

        # check task status
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            for client in clientList:
                if client.isDone() is not True:
                    continue
            break

        rospy.loginfo("All task is Done!")

    except rospy.ROSInterruptException:
        pass

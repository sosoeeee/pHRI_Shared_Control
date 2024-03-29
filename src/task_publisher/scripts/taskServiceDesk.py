#!/usr/bin/env python3
import sys
import os

# Add the current directory to the path module, in order to import the LocalPlannerClass
sys.path.append(os.path.dirname(__file__))

import rospy
from taskClients import *
from actuator.srv import *


# read task from yaml file, and instantiate them all
if __name__ == '__main__':
    try:
        rospy.init_node('task_service_desk', anonymous=False)

        taskList = rospy.get_param("/task_list", [])

        taskPoolSet = []          # tasks sharing the same id will in the same task pool
        curId = 1
        taskPool = []
        for task in taskList:
            if task['task_id'] != curId:
                taskPoolSet.append(taskPool)
                taskPool = [task]
                curId += 1
            else:
                taskPool.append(task)
        taskPoolSet.append(taskPool)

        poolIdx = 1
        for taskPool in taskPoolSet:
            clientList = []
            for task in taskPool:
                # run task
                if task['task_type'] == 'initRobot':
                    rospy.wait_for_service('initRobot')
                    try:
                        init = rospy.ServiceProxy('initRobot', initRobot)
                        res = init().isReady
                    except rospy.ServiceException as e:
                        rospy.loginfo("Service call failed: %s" % e)

                # following task types can run in parallel
                if task['task_type'] == 'reachGoal':
                    clientList.append(PubGoalActionClient(task))
                if task['task_type'] == 'followPath':
                    clientList.append(PubPathActionClient(task))
                if task['task_type'] == 'followTraj':
                    clientList.append(PubTrajActionClient(task))

            for client in clientList:
                client.sendReq()

            # check task status
            while not rospy.is_shutdown():
                rospy.sleep(0.1)
                if all([client.isDone() for client in clientList]):
                    rospy.loginfo("All tasks in task pool %d is Done!" % poolIdx)
                    poolIdx += 1
                    break
    
            rospy.sleep(3) # wait controller switch to deactivating mode
            
        # rospy.spin()

    except rospy.ROSInterruptException:
        pass

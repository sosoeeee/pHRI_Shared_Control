#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped

class EnvPublisher:
    def __init__(self):
        # get the parameters
        self.VICON_topics = rospy.get_param('~env_publisher/VICON_topics', [])
        self.obstacle_args = rospy.get_param('~env_publisher/obstacle_args', [])
        self.publish_rate = rospy.get_param('~env_publisher/publish_rate', 10)
        self.publish_topic = rospy.get_param('~env_publisher/publish_topic', 'env_obstacles')

        if len(self.VICON_topics) != len(self.obstacle_args):
            raise Exception('Number of VICON topics and obstacle arguments do not match')

        # subscribe to the VICON data
        for topic, args in zip(self.VICON_topics, self.obstacle_args):
            rospy.Subscriber(topic, TransformStamped, self.updateObstacles, args, queue_size=1)

        # initialize the obstacles
        self.typeDict = {
            'cube': Marker.CUBE,
            'sphere': Marker.SPHERE,
            'cylinder': Marker.CYLINDER,
            'arrow': Marker.ARROW,
        }
        self.obstacles = []
        self.initObstacles(len(self.VICON_topics))

        # publish the markers
        self.env_publisher = rospy.Publisher(self.publish_topic, MarkerArray, queue_size=1)
        self.rate = rospy.Rate(self.publish_rate)

    def initObstacles(self, num):
        for i in range(num):
            marker = Marker()
            marker.header.frame_id = "map" # need create a frame for visual(map is defaulf one)
            marker.ns = "env"
            marker.id = i
            marker.type = self.typeDict[self.obstacle_args[i]['type']]
            marker.action = Marker.ADD

            # shape parameters
            if self.obstacle_args[i]['type'] == 'sphere':
                marker.scale.x = self.obstacle_args[i]['radius']
                marker.scale.y = self.obstacle_args[i]['radius']
                marker.scale.z = self.obstacle_args[i]['radius']

            marker.color.r = 1
            marker.color.g = 0
            marker.color.b = 0
            marker.color.a = 1
            self.obstacles.append(marker)

    def updateObstacles(self, data, args):
        i = args['id']
        self.obstacles[i].pose.position.x = data.transform.translation.x
        self.obstacles[i].pose.position.y = data.transform.translation.y
        self.obstacles[i].pose.position.z = data.transform.translation.z
        self.obstacles[i].pose.orientation.x = data.transform.rotation.x
        self.obstacles[i].pose.orientation.y = data.transform.rotation.y
        self.obstacles[i].pose.orientation.z = data.transform.rotation.z
        self.obstacles[i].pose.orientation.w = data.transform.rotation.w

    def run(self):
        # test Rviz
        # self.updateObstacles(None, {'id': 0})

        while not rospy.is_shutdown():
            self.env_publisher.publish(self.obstacles)
            self.rate.sleep()

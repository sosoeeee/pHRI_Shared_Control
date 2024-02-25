#!/usr/bin/env python3
import numpy as np
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped
import tf


class EnvPublisher:
    def __init__(self):
        # get the parameters
        self.VICON_topics = rospy.get_param('/env_publisher/VICON_topics', [])
        self.VICON_transform = rospy.get_param('/env_publisher/VICON_transform',
                                               {'x': 0, 'y': 0, 'z': 0, 'qx': 0, 'qy': 0, 'qz': 0, 'qw': 1})
        self.obstacle_args = rospy.get_param('/env_publisher/obstacle_args', [])
        self.publish_rate = rospy.get_param('/env_publisher/publish_rate', 10)
        self.publish_topic = rospy.get_param('env_publisher/publish_topic', 'env_obstacles')
        self.world_frame = rospy.get_param('/world_frame', 'map')

        if len(self.VICON_topics) != len(self.obstacle_args):
            raise Exception('Number of VICON topics and obstacle arguments do not match')

        # subscribe to the VICON data
        for topic, args in zip(self.VICON_topics, self.obstacle_args):
            rospy.Subscriber(topic, TransformStamped, self.updateObstacles, args, queue_size=1)

        # publish the transform between the map and the VICON frame
        self.br = tf.TransformBroadcaster()

        # initialize the obstacles
        self.typeDict = {
            'cube': Marker.CUBE,
            'sphere': Marker.SPHERE,
            'cylinder': Marker.CYLINDER,
            'arrow': Marker.ARROW,
        }
        self.obstacles = MarkerArray()
        self.obstacles.markers = []
        self.initObstacles(len(self.VICON_topics))

        # publish the markers
        self.env_publisher = rospy.Publisher(self.publish_topic, MarkerArray, queue_size=1)
        self.rate = rospy.Rate(self.publish_rate)

    def initObstacles(self, num):
        for i in range(num):
            marker = Marker()
            marker.header.frame_id = "VICON"
            marker.ns = "env"
            marker.id = i
            marker.type = self.typeDict[self.obstacle_args[i]['type']]
            marker.action = Marker.ADD

            # shape parameters
            if self.obstacle_args[i]['type'] == 'sphere':
                marker.scale.x = self.obstacle_args[i]['radius']
                marker.scale.y = self.obstacle_args[i]['radius']
                marker.scale.z = self.obstacle_args[i]['radius']
            
            if self.obstacle_args[i]['type'] == 'cube':
                marker.scale.x = self.obstacle_args[i]['size_x']
                marker.scale.y = self.obstacle_args[i]['size_y']
                marker.scale.z = self.obstacle_args[i]['size_z']

            marker.color.r = 1
            marker.color.g = 0
            marker.color.b = 0
            marker.color.a = 1
            self.obstacles.markers.append(marker)

    def updateObstacles(self, data, args):
        i = args['id']
        self.obstacles.markers[i].pose.position.x = data.transform.translation.x
        self.obstacles.markers[i].pose.position.y = data.transform.translation.y
        self.obstacles.markers[i].pose.position.z = data.transform.translation.z
        self.obstacles.markers[i].pose.orientation.x = data.transform.rotation.x
        self.obstacles.markers[i].pose.orientation.y = data.transform.rotation.y
        self.obstacles.markers[i].pose.orientation.z = data.transform.rotation.z
        self.obstacles.markers[i].pose.orientation.w = data.transform.rotation.w

    # debug function
    def generateRandomObstacles(self, num):
        vertex1 = np.array([0, 0, 0])
        vertex2 = np.array([5, 5, 5])
        for i in range(num):
            random_vertex = np.random.rand(3) * (vertex2 - vertex1) + vertex1
            self.obstacles.markers[i].pose.position.x = random_vertex[0]
            self.obstacles.markers[i].pose.position.y = random_vertex[1]
            self.obstacles.markers[i].pose.position.z = random_vertex[2]

    def run(self):
        # test Rviz
        # self.updateObstacles(None, {'id': 0})

        # debug
        index = 0

        while not rospy.is_shutdown():
            self.env_publisher.publish(self.obstacles)

            self.br.sendTransform((self.VICON_transform['x'], self.VICON_transform['y'], self.VICON_transform['z']),
                                  (self.VICON_transform['qx'], self.VICON_transform['qy'], self.VICON_transform['qz'],
                                   self.VICON_transform['qw']),
                                  rospy.Time.now(),
                                  "VICON",
                                  self.world_frame)

            # debug
            if index % 1000 == 0:
                self.generateRandomObstacles(len(self.VICON_topics))
                index = 0
            index += 1

            self.rate.sleep()


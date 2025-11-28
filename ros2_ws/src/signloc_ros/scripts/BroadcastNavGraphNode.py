#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

import numpy as np
from collections import OrderedDict
from std_msgs.msg import UInt16, Float32MultiArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
import networkx as nx
from sensor_msgs.msg import NavSatFix
from geopy.geocoders import Nominatim
import utm
import pickle

class BroadcastNavGraphNode(Node):

    def __init__(self):
        super().__init__('BroadcastNavGraphNode')


        self.declare_parameter('navGraphPath', "") 
        navGraphPath = self.get_parameter('navGraphPath').value
        self.get_logger().info(f'The value of navGraphPath is: {navGraphPath}')

        self.declare_parameter('latlonTopic', "") 
        latlonTopic = self.get_parameter('latlonTopic').value
        self.get_logger().info(f'The value of latlonTopic is: {latlonTopic}')

        self.G = pickle.load(open(navGraphPath, 'r+b'))

        self.init_gps = False

    
        self.marker_pub = self.create_publisher(MarkerArray, "navGraphTopic", 10)
        self.subscription = self.create_subscription(NavSatFix, latlonTopic, self.listener_callback, 10)
           


        # nav_msg = rclpy.wait_for_message(NavSatFix, self, latlonTopic, timeout_sec=100)
        # if nav_msg:
        #     lat = nav_msg.latitude 
        #     lon = nav_msg.longitude 
        # self.wp = utm.from_latlon(lat, lon) 
        # self.markers = self.createMarkers()


        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('BroadcastNavGraphNode::Ready!')


    def listener_callback(self, nav_msg):

        if not self.init_gps:
            lat = nav_msg.latitude 
            lon = nav_msg.longitude 
            self.wp = utm.from_latlon(lat, lon)
            self.markers = self.createMarkers()
            self.init_gps = True




    def timer_callback(self):

        if not self.init_gps:
            return 

        markerArray = MarkerArray()
        markerArray.markers = self.markers
        # Renumber the marker IDs
        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1

        # Publish the MarkerArray
        self.marker_pub.publish(markerArray)


    
    def createMarkers(self):

        markers = []

        for n, d in self.G.nodes(data=True):
            if d["type"] in ["intersection", "terminal", "room", "portal", "building"]:

                pos = self.G.nodes[n]["pos"]
                marker = Marker()
                marker.header.frame_id = "map"
                marker.action = marker.ADD
                marker.type = marker.SPHERE
                marker.pose.position.x = pos[0] - self.wp[0]
                marker.pose.position.y = pos[1] - self.wp[1]
                marker.pose.position.z = 5.0 * pos[2]
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                scale = 1.0

                if d["type"] in ["room"]:
                    continue
                    color = [1.0, 0.0, 0.0]

                elif d["type"] in ["intersection", "terminal"]:
                    color = [1.0, 0.0, 1.0]
                    scale = 2.0
                    # if pos[2] < 3:
                    #     continue
                    #     color = [0.2, 0.2, 0.8]
                    #     scale = 1

                elif d["type"] in ["portal"]:
                    color = [0.0, 0.0, 1.0]

                # elif d["type"] in ["building"]:
                #     color = [0.2, 0.2, 0.2]

    
                marker.scale.x = scale
                marker.scale.y = scale
                marker.scale.z = scale
                marker.color.a = 0.8
                marker.color.r = color[2]
                marker.color.g = color[1]
                marker.color.b = color[0]
                marker.lifetime = rclpy.duration.Duration(seconds=3.0).to_msg()

                markers.append(marker)

        for n, d in self.G.nodes(data=True):
            if d["type"] in ["intersection", "terminal"]:

                pos = self.G.nodes[n]["pos"]
                marker = Marker()
                marker.header.frame_id = "map"
                marker.action = marker.ADD
                marker.type = marker.TEXT_VIEW_FACING
                marker.pose.position.x = pos[0] - self.wp[0]
                marker.pose.position.y = pos[1] - self.wp[1]
                marker.pose.position.z = 5.0 * pos[2]
                marker.pose.orientation.x = 0.0 
                marker.pose.orientation.y = 0.0 
                marker.pose.orientation.z = 0.0 
                marker.pose.orientation.w = 1.0 
                marker.text = str(n)
                scale = 4.0
                color = [0.0, 0.0, 0.0]               
                marker.scale.z = scale
                marker.color.a = 1.0
                marker.color.r = color[2]
                marker.color.g = color[1]
                marker.color.b = color[0]
                marker.lifetime = rclpy.duration.Duration(seconds=3.0).to_msg()
                #rospy.loginfo(str(n))

                #markers.append(marker)


        marker = Marker()
        marker.header.frame_id = "map"
        marker.action = marker.ADD
        marker.type = marker.LINE_LIST
        color = [0.0, 0.0, 0.0]
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 0.8
        marker.color.r = color[2]
        marker.color.g = color[1]
        marker.color.b = color[0]
        marker.lifetime = rclpy.duration.Duration(seconds=3.0).to_msg()
        points = []

        for u, v, d in self.G.edges(data=True):

            if d["type"] in ["action"]:

                u1 = self.G.nodes[u]["pos"]
                v1 = self.G.nodes[v]["pos"]

                # if u1[2] != 3.0:
                #     continue

                p1 = Point()
                p1.x = u1[0] - self.wp[0]
                p1.y = u1[1] - self.wp[1]
                p1.z = 5.0 * u1[2]
                p2 = Point()
                p2.x = v1[0] - self.wp[0]
                p2.y = v1[1] - self.wp[1]
                p2.z = 5.0 * v1[2]
                points.append(p1)
                points.append(p2)
               
              
        marker.points = points
        markers.append(marker)

        return markers
   

def main(args=None):
    rclpy.init(args=args)

    broadcastNavGraphNode = BroadcastNavGraphNode()

    rclpy.spin(broadcastNavGraphNode)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    broadcastNavGraphNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
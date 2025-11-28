#!/usr/bin/env python3

import numpy as np
from collections import OrderedDict
import rospy
from std_msgs.msg import UInt16, Float32MultiArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
import networkx as nx
import pickle
from geopy.geocoders import Nominatim
import utm
from sensor_msgs.msg import NavSatFix

class BroadcastNavGraphNode():

    def __init__(self):


        self.marker_pub = rospy.Publisher("navGraphTopic", MarkerArray,  queue_size=10)


        navGraphPath = rospy.get_param('navGraphPath')

        latlonTopic = rospy.get_param('latlonTopic')

        nav_msg = rospy.wait_for_message(latlonTopic, NavSatFix, timeout=100)
        lat = nav_msg.latitude 
        lon = nav_msg.longitude 

        #address = rospy.get_param('address')

        self.G = pickle.load(open(navGraphPath, 'r+b'))

        # geolocator = Nominatim(user_agent="orienternet")
        # location = geolocator.geocode(address)
        # lat, lon = (location.latitude, location.longitude)
        self.wp = utm.from_latlon(lat, lon) 
       

        rospy.loginfo("BroadcastNavGraphNode::Ready!")

        markers = self.createMarkers()

        while not rospy.is_shutdown():

            markerArray = MarkerArray()
            markerArray.markers = markers
            # Renumber the marker IDs
            id = 0
            for m in markerArray.markers:
                m.id = id
                id += 1

            # Publish the MarkerArray
            self.marker_pub.publish(markerArray)

            rospy.sleep(0.1)




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
                marker.pose.position.z = 5 * pos[2]
                marker.pose.orientation.x = 0 
                marker.pose.orientation.y = 0 
                marker.pose.orientation.z = 0 
                marker.pose.orientation.w = 1 
                scale = 1

                if d["type"] in ["room"]:
                    continue
                    color = [1.0, 0.0, 0.0]

                elif d["type"] in ["intersection", "terminal"]:
                    color = [1.0, 0.0, 1.0]
                    scale = 2
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
                marker.lifetime = rospy.Duration(3)

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
                marker.pose.position.z = 5 * pos[2]
                marker.pose.orientation.x = 0 
                marker.pose.orientation.y = 0 
                marker.pose.orientation.z = 0 
                marker.pose.orientation.w = 1 
                marker.text = str(n)
                scale = 4
                color = [0.0, 0.0, 0.0]               
                marker.scale.z = scale
                marker.color.a = 1.0
                marker.color.r = color[2]
                marker.color.g = color[1]
                marker.color.b = color[0]
                marker.lifetime = rospy.Duration(3)
                #rospy.loginfo(str(n))

                #markers.append(marker)


        marker = Marker()
        marker.header.frame_id = "map"
        marker.action = marker.ADD
        marker.type = marker.LINE_LIST
        color = [0.0, 0.0, 0.0]
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.a = 0.8
        marker.color.r = color[2]
        marker.color.g = color[1]
        marker.color.b = color[0]
        marker.lifetime = rospy.Duration(3)
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
                p1.z = 5 * u1[2]
                p2 = Point()
                p2.x = v1[0] - self.wp[0]
                p2.y = v1[1] - self.wp[1]
                p2.z = 5 * v1[2]
                points.append(p1)
                points.append(p2)
               
              
        marker.points = points
        markers.append(marker)

        return markers

def main():
    

    rospy.init_node('BroadcastNavGraphNode', anonymous=True)
    omn = BroadcastNavGraphNode()
    rospy.spin()

if __name__ == "__main__":

    main()
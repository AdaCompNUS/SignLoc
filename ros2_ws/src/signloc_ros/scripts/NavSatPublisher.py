#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geopy.geocoders import Nominatim
import numpy as np
import utm
from signloc_msgs.msg import UTM


class NavSatPublisher(Node):

    def __init__(self):
        super().__init__('navSatPublisher')

        # self.declare_parameter('latlonCoords', [1.2945764, 103.7756446]) 
        # latlonCoords = self.get_parameter('latlonCoords').value
        # self.get_logger().info(f'The value of latlonCoords is: {latlonCoords}')

        self.declare_parameter('latlonTopic', "") 
        latlonTopic = self.get_parameter('latlonTopic').value
        self.get_logger().info(f'The value of latlonTopic is: {latlonTopic}')

        self.declare_parameter('utmTopic', "") 
        utmTopic = self.get_parameter('utmTopic').value
        self.get_logger().info(f'The value of utmTopic is: {utmTopic}')

        self.lat, self.lon = 1.2945764, 103.7756446
        self.wp = utm.from_latlon(self.lat, self.lon) 

        self.gps_pub = self.create_publisher(NavSatFix, latlonTopic, 10)
        self.utm_pub = self.create_publisher(UTM, utmTopic, 10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('NavSatPublisher::Ready!')


       

    def timer_callback(self):

        nav_msg = NavSatFix()
        #nav_msg.header.seq = 999
        nav_msg.header.stamp.sec = 1435607489
        nav_msg.header.stamp.nanosec = 924811809
        nav_msg.header.frame_id = "base_link"
        nav_msg.status.status = 0
        nav_msg.status.service = 1

        nav_msg.latitude = self.lat
        nav_msg.longitude = self.lon
        nav_msg.altitude = 0.551
        nav_msg.position_covariance = np.array([3.9561210000000004, 0.0, 0.0, 0.0, 3.9561210000000004, 0.0, 0.0, 0.0, 7.650756])
        nav_msg.position_covariance_type = 2

        utm_msg = UTM()
        utm_msg.lat = self.wp[0]
        utm_msg.lon = self.wp[1]
        utm_msg.header.stamp.sec = 1435607489
        utm_msg.header.stamp.nanosec = 924811809
        utm_msg.header.frame_id = "base_link"


        self.gps_pub.publish(nav_msg)
        self.utm_pub.publish(utm_msg)

        #self.get_logger().info('NavSatPublisher::Publish!')

        #self.get_logger().info('Publishing: "%s"' % msg.data)



def main(args=None):
    rclpy.init(args=args)

    navSatPublisher = NavSatPublisher()

    rclpy.spin(navSatPublisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    navSatPublisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
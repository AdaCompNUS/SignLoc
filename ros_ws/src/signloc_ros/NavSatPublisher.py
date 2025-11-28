#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from geopy.geocoders import Nominatim
import numpy as np
import utm
from sign_msgs.msg import UTM

def NavSatPublisher():
    
    rospy.init_node('NavSatPublisher', anonymous=True)
    latlonTopic = rospy.get_param('latlonTopic')
    utmTopic = rospy.get_param('utmTopic')

    gps_pub = rospy.Publisher(latlonTopic, NavSatFix, queue_size=10)
    utm_pub = rospy.Publisher(utmTopic, UTM, queue_size=10)

    # address = rospy.get_param('address')
    # geolocator = Nominatim(user_agent="orienternet")
    # location = geolocator.geocode(address)
    # lat, lon = (location.latitude, location.longitude)
    # rospy.loginfo("{}, {}".format(lat, lon))

    lat, lon = 1.2945764, 103.7756446
    wp = utm.from_latlon(lat, lon) 


    rospy.loginfo("NavSatPublisherNode::Ready!")

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        nav_msg = NavSatFix()
        nav_msg.header.seq = 999
        nav_msg.header.stamp.secs = 1435607489
        nav_msg.header.stamp.nsecs = 924811809
        nav_msg.header.frame_id = "base_link"
        nav_msg.status.status = 0
        nav_msg.status.service = 1

        nav_msg.latitude = lat
        nav_msg.longitude = lon
        nav_msg.altitude = 0.551
        nav_msg.position_covariance = np.array([3.9561210000000004, 0.0, 0.0, 0.0, 3.9561210000000004, 0.0, 0.0, 0.0, 7.650756])
        nav_msg.position_covariance_type = 2

        utm_msg = UTM()
        utm_msg.lat = wp[0]
        utm_msg.lon = wp[1]
        utm_msg.header.stamp.secs = 1435607489
        utm_msg.header.stamp.nsecs = 924811809
        utm_msg.header.frame_id = "base_link"


        #rospy.loginfo(hello_str)
        gps_pub.publish(nav_msg)
        utm_pub.publish(utm_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        NavSatPublisher()
    except rospy.ROSInterruptException:
        pass
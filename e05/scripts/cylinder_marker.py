#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker

rospy.init_node('cylinder_marker_node')
pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

marker = Marker()
marker.header.frame_id = "base_link"
marker.header.stamp = rospy.Time.now()
marker.ns = "cylinder"
marker.id = 0
marker.type = Marker.CYLINDER
marker.action = Marker.ADD

marker.pose.position.x = 0.715
marker.pose.position.y = 0
marker.pose.position.z = 0.55
marker.pose.orientation.w = 1.0

marker.scale.x = 0.05
marker.scale.y = 0.05
marker.scale.z = 0.3

marker.color.r = 0.0
marker.color.g = 0.5
marker.color.b = 1.0
marker.color.a = 1.0

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    pub.publish(marker)
    rate.sleep()

#!/usr/bin/env python
from __future__ import print_function
import rospy
import std_msgs.msg as rosmsg
import nav_msgs.msg as navmsg
import sensor_msgs.msg as senmsg
#import pyqtgraph as pg


def utmCallback(msg):
    print("UTM zone: %s" %  (msg.data))

def odometryCallBack(msg):
    print("odom: %.4f %.4f " % (msg.pose.pose.position.x, msg.pose.pose.position.y))

def gpsFixCallBack(msg):
    print("gps: %.4f %.4f " % (msg.latitude, msg.longitude))

def listener():

    # In ROS, nodes are uniquely named. The anonymous=True flag means that rospy will choose a unique
    # name for our "listener" node so that multiple listeners can run simultaneously.
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber('odom', navmsg.Odometry, odometryCallBack)
    rospy.Subscriber("gps/utmzone", rosmsg.String, utmCallback)
    rospy.Subscriber("gps/fix", senmsg.NavSatFix, gpsFixCallBack)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    print(__file__, "- gps message reader started ... ")
    listener()

#!/usr/bin/env python
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class loadWaypoints():

    def __init__(self):

        self.freq = 1 # 1 Hz
        self.filename = rospy.get_param('~WaypointFile','waypoints.txt')
        self.wptPub = rospy.Publisher("waypoints", Path, latch=True, queue_size=1)

        while not rospy.is_shutdown():
            # Create a waypoint path only once
            try:
                waypointPath
            except NameError:
                waypointPath = loadWpts(self.filename)
                rospy.loginfo("Loaded Waypoints: %s",self.filename)
            self.wptPub.publish(waypointPath)
            rospy.sleep(self.freq)


def loadWpts(filename):
# generates a path message from a waypoint file
    waypointPath = Path()
    waypointPath.header.stamp = rospy.Time.now()
    waypointPath.header.frame_id = "map"
    cx,cy = [], []
    with open(filename) as f:
        for line in f:
	
            rx, ry = map(float,line.split(','))
            cx.append(rx)
            cy.append(ry)

    #x, y, _, _, _ = CubicSplinePlanner.calc_spline_course(cx,cy,ds=1)
    x,y = cx,cy
    for i in range(len(x)):

        cpose = PoseStamped()
        cpose.header.stamp = rospy.Time.now()
        cpose.header.frame_id = "map"
        cpose.pose.position.x, cpose.pose.position.y = x[i],y[i]
        cpose.pose.position.z = 0.0
        cpose.pose.orientation.x = 0
        cpose.pose.orientation.y = 0
        cpose.pose.orientation.z = 0
        cpose.pose.orientation.w = 1
        waypointPath.poses.append(cpose)

    return waypointPath

if __name__ == '__main__':
    rospy.init_node('WaypintLoader')
theNode = loadWaypoints()
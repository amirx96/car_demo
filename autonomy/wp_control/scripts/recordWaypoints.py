#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
class WaypointGen():

    def PoseCallback(self, data):
        self.position = data.pose.pose.position
        #print(self.position)
        self.position_received = True

    def saveFile(self):
        # Write it to a file
        rospy.logwarn("Writing to File %s",self.filename)
        f = open(self.filename, 'w')
        for i in range(len(self.points)):
            if i == 0:
                k = 1
            if i == len(self.points) - 1:
                k = i -1
            else:
                k = i
            f.write(str(self.points[i].x) + ',' + str(self.points[i].y) +  '\n')
        f.close()
        

    def __init__(self):
        self.position = Point()
        self.position_received = False
        self.wptDist = rospy.get_param('~WaypointDistance', 1)
    
        print self.wptDist
        self.filename = rospy.get_param('~WaypointFile','waypoints.txt')
        print self.filename
        rospy.logwarn('Waypoint Recorder Initialized. Point Seperation Distance %f, File name %s',self.wptDist,self.filename)
        self.stop = 0
        rospy.Subscriber("/base_pose_ground_truth", Odometry, self.PoseCallback)
        self.wptPub = rospy.Publisher("waypoints", Path, latch=True, queue_size=1)
        self.points = []
        self.cur_steering = 0
        self.wpt_path = Path()
        while not rospy.is_shutdown():
    
            if self.position_received == True:
                #print(len(self.points))
                if len(self.points) < 1:
                    rospy.loginfo('grabbed first position')
                    rospy.loginfo(str(self.position.x) + ',' + str(self.position.y))
                    self.points.append(self.position)

                if dist2D(self.points[-1],self.position) > self.wptDist:
                    self.points.append(self.position)
                    curpose = PoseStamped()
                    curpose.pose.position = self.position
                    curpose.header.stamp = rospy.Time.now()
                    self.wpt_path.poses.append(curpose)
                    self.wpt_path.header.stamp = rospy.Time.now()
                    self.wpt_path.header.frame_id = "map"
                    self.wptPub.publish(self.wpt_path)
                    rospy.loginfo('Added waypoint x: %f, y: %f',self.position.x,self.position.y)
        
        rospy.on_shutdown(self.saveFile)



def dist2D(pt1, pt2):
   # print(((pt2.x - pt1.x)**2.0 + (pt2.y - pt1.y)**2.0)**(0.5))
    return ((pt2.x - pt1.x)**2.0 + (pt2.y - pt1.y)**2.0)**(0.5)

if __name__ == '__main__':
    rospy.init_node('RecordWaypoints')
theNode = WaypointGen()

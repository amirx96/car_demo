#!/usr/bin/env python
import math
import time

import numpy as np
import rospy
# from control_simulation.msg import lateral_debug
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker
from prius_msgs.msg import Control
import purePursuit
import SpeedPID


class ControlNode():

    def odomCallback(self, data):
        """ Pose Update Callback """
        self.xpos = data.pose.pose.position.x
        self.ypos = data.pose.pose.position.y
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(
            [data.pose.pose.orientation.x, data.pose.pose.orientation.y,
             data.pose.pose.orientation.z, data.pose.pose.orientation.w])
        self.velocity = (data.twist.twist.linear.x**2 +
                         data.twist.twist.linear.y**2)**0.5
        self.angular_velocity = data.twist.twist.angular.z

    def waypointCallback(self, data):
        """ Waypoint Call Back """
        self.waypoint_path = data

    def genGoalMarker(self, gx, gy):
        """ Generates Goal Marker for RViz """
        self.goalMarker.header.frame_id = "map"
        self.goalMarker.header.stamp = rospy.Time.now()
        self.goalMarker.type = self.goalMarker.SPHERE
        self.goalMarker.action = self.goalMarker.ADD
        self.goalMarker.scale.x = 0.75
        self.goalMarker.scale.y = 0.75
        self.goalMarker.scale.z = 0.75
        self.goalMarker.pose.position.x = gx
        self.goalMarker.pose.position.y = gy
        self.goalMarker.pose.position.z = 0
        self.goalMarker.pose.orientation.x = 0
        self.goalMarker.pose.orientation.y = 0
        self.goalMarker.pose.orientation.z = 0
        self.goalMarker.pose.orientation.w = 1
        self.goalMarker.color.a = 1
        self.goalMarker.color.r = 1
        self.goalMarker.lifetime = rospy.Duration()

    def __init__(self):
        # Initiate variables
        self.control_type = rospy.get_param(
            '~lat_control', 'stanley')  # Default is stanley controller
        rospy.loginfo("Setting up Control Node")
        rospy.loginfo("Lateral Control Type: %s", self.control_type)
        rospy.sleep(1)
        self.goalMarker = Marker()
        self.freq = 1/30.0  # 30 hz
        self.velocity = 0
        self.angular_velocity = 0
        self.xpos = 0
        self.ypos = 0
        self.roll = 0
        self.yaw = 0
        self.prev_the = 0
        self.prev_e_lat = 0
        self.pitch = 0
        self.steer_cmd = 0.0
        self.waypoint_path = Path()
        self.waypoint_path_cubic_spline = Path()
        self.path_changed = True
        self.prev_cmd = 0.0

        # Publishers
        self.goalMarkerPub = rospy.Publisher(
            "~goal", Marker, queue_size=10)
        self.cmdPub = rospy.Publisher(
            "/prius", Control, latch=True,)

        # Subscribers
        rospy.Subscriber("/base_pose_ground_truth",
                         Odometry, self.odomCallback)
        rospy.Subscriber("/waypoints",
                         Path, self.waypointCallback)
        rospy.sleep(1)

        # Setup Controller
        lat_control = purePursuit.PurePursuit(path=self.waypoint_path)
        speed_control = SpeedPID.SpeedPID(path=self.waypoint_path,target_speed=3)

        r = rospy.Rate(1/self.freq)
        # Control Loop
        while not rospy.is_shutdown():

            lat_control.update(self)  # Update Control
            speed_control.update(self,target_speed=3)

            # Build Command
            self.prius_command = Control()
            self.prius_command.steer = np.clip(lat_control.getSteeringCmd(), -1, 1)
            self.prius_command.throttle = np.clip(speed_control.get_pedal_cmds()[0],0,1)
            self.prius_command.brake = np.clip(speed_control.get_pedal_cmds()[1],0,1)

            # Update Throttle
            self.prius_command.shift_gears = 2

            self.cmdPub.publish(self.prius_command)

            rospy.loginfo("Steer CMD %f, Throttle CMD %f, Brake CMD %f", self.prius_command.steer, self.prius_command.throttle, self.prius_command.brake)
            gx, gy = lat_control.getGoalPoint()
            self.genGoalMarker(gx, gy)
            self.goalMarkerPub.publish(self.goalMarker)

            r.sleep()


if __name__ == '__main__':
    rospy.init_node('lateral_control')
theNode = ControlNode()

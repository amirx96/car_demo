import math

import numpy as np
import rospy
from nav_msgs.msg import Path


class PurePursuit():
    def __init__(self, path, k=0.12, lfc=3, kp=1.0, L=2.9):
        self.current_target_idx = 0
        self.last_target_idx = 0
        self.e_lat = 0
        self.curvature = 0
        self.gx = self.gy = 0
        self.path_valid = False
        self.cx, self.cy = Path2Array(path)
        self.path_valid = (len(self.cx) > 1)
        self.k = k  # look forward gain
        self.Lfc = lfc  # look-ahead
        self.Kp = kp  # speed propotional gain
        self.L = L  # [m] wheel base of vehicle

    def calc_target_idx(self):
        fx = self.xpos + 1 * (self.L/2)*np.cos(self.yaw) # I think the origin of "/base_pose_ground_truth" is at the front wheels.
        fy = self.ypos + 1 * (self.L/2)*np.sin(self.yaw)
        fr_axle_vec = [np.sin(self.yaw), -np.cos(self.yaw)]
        # Reindex if our last target index gives an error larger than 5 meters.
        reindex_required = (np.sqrt(
            (fx - self.cx[self.last_target_idx])**2 + (fy - self.cy[self.last_target_idx])**2) > 5)
        L = 0.0
        if reindex_required:
            dx = [fx - icx for icx in self.cx]
            dy = [fy - icy for icy in self.cy]
            d = [np.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
            error = min(d)
            target_idx = d.index(error)
            error_vec = [dx[target_idx], dy[target_idx]]
            e_lat = np.dot(fr_axle_vec, error_vec)
        else:
            search_range = 10
            start_idx = self.last_target_idx - \
                search_range if self.last_target_idx >= search_range else 0
            end_idx = self.last_target_idx + \
                search_range if self.last_target_idx <= len(
                    self.cx) - search_range else len(self.cx)
            rcx = self.cx[start_idx:end_idx]
            rcy = self.cy[start_idx:end_idx]

            dx = [fx - icx for icx in rcx]
            dy = [fy - icy for icy in rcy]
            d = [np.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
            error = min(d)
            r_target_idx = d.index(error)
            error_vec = [dx[r_target_idx], dy[r_target_idx]]
            target_idx = r_target_idx + start_idx
            e_lat = np.dot(fr_axle_vec, error_vec)
        Lf = self.velocity*self.k + self.Lfc
        while Lf > L and (target_idx + 1) < len(self.cx):
            dx = self.cx[target_idx + 1] - self.cx[target_idx]
            dy = self.cy[target_idx + 1] - self.cy[target_idx]
            L += math.sqrt(dx ** 2 + dy ** 2)
            target_idx += 1
        self.current_target_idx = target_idx
        self.e_lat = e_lat
        return

    def pure_pursuit_control(self):
        self.calc_target_idx()
        ind = self.current_target_idx
        pind = self.last_target_idx

        if self.last_target_idx >= ind:
            ind = pind

        if ind < len(self.cx):
            tx = self.cx[ind]
            ty = self.cy[ind]
        else:
            tx = self.cx[-1]
            ty = self.cy[-1]
            ind = len(self.cx) - 1

        # Curvature Moving Average Filter
        curv_move_n = 30
        start_avg_idx = 0 + ind
        if start_avg_idx < len(self.cx) - curv_move_n:
            self.curvature = getCurvature(self.cx[start_avg_idx:(
                start_avg_idx+curv_move_n)], self.cy[start_avg_idx:(start_avg_idx+curv_move_n)])
        else:
            self.curvature = 0

        alpha = math.atan2(ty - self.ypos, tx - self.xpos) - self.yaw

        if self.velocity < 0:  # backwards travel
            alpha = math.pi - alpha

        Lf = self.k * self.velocity + self.Lfc

        d_pp = math.atan2(2.0 * self.L * math.sin(alpha) /
                          Lf, 1.0)  # PurePursuit Term

        self.delta = d_pp
        self.last_target_idx = self.current_target_idx
        return

    def update(self, state):
        self.xpos = state.xpos
        self.ypos = state.ypos
        (self.roll, self.pitch, self.yaw) = state.roll, state.pitch, state.yaw
        self.velocity, self.angular_velocity = state.velocity, state.angular_velocity
        self.control_freq = state.freq

        if self.path_valid:

            self.pure_pursuit_control()  # Pure Pursuit Update

            self.gx = self.cx[self.current_target_idx]
            self.gy = self.cy[self.current_target_idx]
        else:
            self.gx = 0
            self.gy = 0
            self.delta = 0
            rospy.logwarn("Path Error")
            return
        return

    def getGoalPoint(self):
        return self.gx, self.gy

    def getSteeringCmd(self):
        return self.delta

    def getLatError(self):
        return self.e_lat

    def returnCurvCalculation(self):
        return self.curvature


def Path2Array(path):
    cx = []
    cy = []
    for wpt in path.poses:
        cx.append(wpt.pose.position.x)
        cy.append(wpt.pose.position.y)

    return cx, cy

def getCurvature(x, y):
    """Returns the curvature with 3n points using the Menger Formula"""

    if not (len(x) % 3 == 0):
        print("ERROR, curvature calculation requires 3n points")
        return 0
    if not (len(y) == len(x)):
        print("ERROR, curvature x & y lengths are not the same")
        return 0

    c_avg = []
    for i in range(len(x) - 2):
        x1, y1, x2, y2, x3, y3 = x[i], y[i], x[i+1], y[i+1], x[i+2], y[i+2]
        det_array = np.array([[x1, y1, 1], [x2, y2, 1], [x3, y3, 1]])
        area = np.linalg.det(det_array) / 2  # area of a triangle with 3 points
        denom = (((x1 - x2)**2 + (y1-y2)**2)**0.5) * (((x2 - x3) **
                                                       2 + (y2-y3)**2)**0.5) * (((x3-x1)**2 + (y3-y1)**2)**0.5)
        c_avg.append(4*area/denom)

    curvature = sum(c_avg) / float(len(c_avg))
    return curvature


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

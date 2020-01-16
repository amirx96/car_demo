import math
import numpy as np
import rospy
from nav_msgs.msg import Path


class SpeedPID():

    def __init__(self, path, kp_throttle=0.3, kp_brake=0.15, k=0.12, target_speed=0, max_lat_accel=0.25, L=2.9):
        self.current_target_idx = 0
        self.look_ahead_idx = 0
        self.last_target_idx = 0
        self.curvature = 0
        self.limit_target_velocity = 0
        self.cmd_throttle = 0.0
        self.cmd_brake = 0.0
        self.gx = self.gy = 0
        self.path_valid = False
        self.cx, self.cy = Path2Array(path)
        self.path_valid = (len(self.cx) > 1)
        self.kp_throttle = kp_throttle  # Speed KP for throttle
        self.kp_brake = kp_brake
        self.k = k  # look forward gain
        self.L = L  # [m] wheel base of vehicles
        self.Lfc = 10  # [m] Lookforward constant for curvature calculation
        self.max_lat_accel = max_lat_accel

    def calc_target_idx(self):
        # I think the origin of "/base_pose_ground_truth" is at the front wheels.
        fx = self.xpos + 0 * (self.L/2)*np.cos(self.yaw)
        fy = self.ypos + 0 * (self.L/2)*np.sin(self.yaw)
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
        self.current_target_idx = target_idx
        while Lf > L and (target_idx + 1) < len(self.cx):
            dx = self.cx[target_idx + 1] - self.cx[target_idx]
            dy = self.cy[target_idx + 1] - self.cy[target_idx]
            L += math.sqrt(dx ** 2 + dy ** 2)
            target_idx += 1
        self.look_ahead_idx = target_idx
        self.e_lat = e_lat
        return

    def speed_pid(self):
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

        # Curvature Moving Average Filter for current position
        curv_move_n = 12
        start_avg_idx = 0 + ind
        if start_avg_idx < len(self.cx) - curv_move_n:
            self.cur_pos_curvature = getCurvature(self.cx[start_avg_idx:(
                start_avg_idx+curv_move_n)], self.cy[start_avg_idx:(start_avg_idx+curv_move_n)])
        else:
            self.cur_pos_curvature = 0

        # Curvature Moving Average Filter for look ahead distance
        curv_move_n = 12
        start_avg_idx = 0 + ind
        if start_avg_idx < len(self.cx) - curv_move_n:
            self.lookahead_pos_curvature = getCurvature(self.cx[start_avg_idx:(
                start_avg_idx+curv_move_n)], self.cy[start_avg_idx:(start_avg_idx+curv_move_n)])
        else:
            self.lookahead_pos_curvature = 0

        self.max_curvature = max(
            [self.cur_pos_curvature, self.lookahead_pos_curvature])

        if self.max_curvature*self.velocity**2 > self.max_lat_accel:
            self.limit_target_velocity = (
                self.max_lat_accel/self.max_curvature)**0.5
            rospy.logwarn("Limiting velocity to %f",
                          self.limit_target_velocity)
        else:
            self.limit_target_velocity = self.target_speed
        velocity_error = self.target_speed - self.velocity

        if velocity_error > -1e-2:
            self.cmd_throttle = self.kp_throttle * velocity_error
            self.cmd_brake = 0.0
        else:
            self.cmd_throttle = 0.0
            self.cmd_brake = self.kp_brake * velocity_error
        return

    def update(self, state, target_speed):
        self.target_speed = target_speed
        self.xpos = state.xpos
        self.ypos = state.ypos
        (self.roll, self.pitch, self.yaw) = state.roll, state.pitch, state.yaw
        self.velocity, self.angular_velocity = state.velocity, state.angular_velocity
        self.control_freq = state.freq

        if self.path_valid:

            self.speed_pid()  # Speed PID update

            self.gx = self.cx[self.current_target_idx]
            self.gy = self.cy[self.current_target_idx]
        else:
            self.gx = 0
            self.gy = 0
            self.delta = 0
            rospy.logwarn("Path Error")
            return
        return

    def get_pedal_cmds(self):
        return [self.cmd_throttle, self.cmd_brake]


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

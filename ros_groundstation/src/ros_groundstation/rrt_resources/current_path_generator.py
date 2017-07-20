#!/usr/bin/python

import rospy
from ros_plane.msg import Current_Path
from math import *
import numpy as np

M_PI_F = 3.14159265358979323846
M_PI_2_F = 1.57079632679489661923


def get_full_current_path(wp_list): # takes in list of NED waypoints, returns list of NE points for drawing

    def mod2pi(phi):
        mod = phi % (2 * np.pi)
        return mod

    def map_pos_2pi(phi):
        if phi > 2*pi:
            while phi > 2*pi:
                phi -= 2*pi
        elif phi < 0:
            while phi < 0:
                phi += 2*pi
        return phi

    class path_manager_base:

        # Init function
        def __init__(self, wp_list):

            # Init Params
            self.R_min = rospy.get_param('R_min', 50.0)

            # waypoint 0 should be initial position, waypoint 1 where you want to start flying to
            self.index_a = 1

            # Class members
            self._num_waypoints = 0
            self.i = 0
            self.Va = 15  # dummy value

            # Member objects; initialize waypoints
            self._waypoints = []

            for wp in wp_list:
                new_wp = self.waypoint_temp()
                self._waypoints.append(new_wp)
                self._waypoints[self._num_waypoints].w0 = wp[0]
                self._waypoints[self._num_waypoints].w1 = wp[1]
                self._waypoints[self._num_waypoints].w2 = wp[2]
                self._waypoints[self._num_waypoints].chi_d = wp[3]
                self._waypoints[self._num_waypoints].chi_valid = True
                self._waypoints[self._num_waypoints].Va_d = self.Va
                self._waypoints[self._num_waypoints].land = False
                self._num_waypoints += 1

            self.point_list = []

        # Subclasses
        class waypoint_temp:
            w0 = 0.0
            w1 = 0.0
            w2 = 0.0
            chi_d = 0.0
            chi_valid = True
            Va_d = 0.0
            land = False

        # Class Member Functions

        def current_path_publisher(self, output):
            self.point_list += output  # "publish"

        # functions
        def iterate(self):
            outputs = self.manage()
            self.current_path_publisher(outputs)

        def manage(self):
            output = []

            # Calc distance between waypoints
            start = np.array(
                    [self._waypoints[self.index_a - 1].w0, self._waypoints[self.index_a - 1].w1])
            end = np.array([self._waypoints[self.index_a].w0,
                            self._waypoints[self.index_a].w1])
            dist = np.linalg.norm(start - end)

            if dist < 2 * self.R_min:
                output = self.manage_line()
            else:
                output = self.manage_dubins()

            return output

        def manage_dubins(self):

            now = self._waypoints[self.index_a]
            past = self.waypoint_temp()

            if (self.index_a == 0):
                past = self._waypoints[self._num_waypoints - 1]
            else:
                past = self._waypoints[self.index_a - 1]


            delta = 1 # m
            points = self.dubins_points(past, now, delta)

            output = []
            for point in points:
                output.append((point[0],point[1]))

            if (self.index_a == (self._num_waypoints - 1)):
                self.index_a = 0
            else:
                self.index_a += 1

            return output

        def rotz(self, theta):
            R = np.array([[cos(theta), -sin(theta), 0.0],
                            [sin(theta),cos(theta),0.0],
                            [0.0, 0.0, 1.0]])
            return R

        def normalize(self, v):
            # Theres probably a better way to do this, but his is what works
            # norm=np.linalg.norm(v, ord=1) # tried this, but it didn't work, result in vector that sums to 1 not magnitude 1
            norm = sqrt(v[0]**2 + v[1]**2 + v[2]**2)

            if norm==0:
                norm=np.finfo(v.dtype).eps

            temp = np.array([0.0, 0.0, 0.0])
            temp[0] = v[0]/norm
            temp[1] = v[1]/norm
            temp[2] = v[2]/norm
            return temp#v/norm

        def dubinsParameters(self, ps, chi_s, pe, chi_e, R):
            dist = np.linalg.norm(ps - pe)
            e1 = np.array([[1], [0], [0]])

            assert (dist >= 2 * R), "waypoints are too close together!"

            crs = ps + R * np.matmul(self.rotz(np.pi/2), np.array([[cos(chi_s)], [sin(chi_s)], [0]]))
            cls = ps + R * np.matmul(self.rotz(-np.pi/2), np.array([[cos(chi_s)], [sin(chi_s)], [0]]))
            cre = pe + R * np.matmul(self.rotz(np.pi/2), np.array([[cos(chi_e)], [sin(chi_e)], [0]]))
            cle = pe + R * np.matmul(self.rotz(-np.pi/2), np.array([[cos(chi_e)], [sin(chi_e)], [0]]))

            # compute length for case 1 rsr
            ang = atan2(cre.item(1)-crs.item(1), cre.item(0)-crs.item(0))
            L1 = np.linalg.norm(crs-cre) + R * mod2pi(2 * np.pi + mod2pi(ang - np.pi / 2) - mod2pi(chi_s - np.pi / 2)) \
                 + R * mod2pi(2 * np.pi + mod2pi(chi_e - np.pi / 2) - mod2pi(ang - np.pi / 2))

            # Compute length for case 2 rsl
            ang = atan2(cle.item(1)-crs.item(1), cle.item(0)-crs.item(0))
            l = np.linalg.norm(cle - crs)

            try:
                ang2 = ang - np.pi / 2 + asin((2 * R) / l)
                L2 = np.sqrt(l ** 2 - 4 * R ** 2) + R * mod2pi(2 * np.pi + mod2pi(ang2) - mod2pi(chi_s - np.pi / 2)) \
                     + R * mod2pi(2 * np.pi + mod2pi(ang2 + np.pi) - mod2pi(chi_e + np.pi / 2))
            except ValueError:
                L2 = 9999

            # Compute length for case 3 lsr
            ang = atan2(cre.item(1)-cls.item(1), cre.item(0)-cls.item(0))
            l = np.linalg.norm(cre-cls)

            try:
                ang2 = acos((2 * R) / l)
                L3 = np.sqrt(l ** 2 - 4 * R ** 2) + R * mod2pi(2 * np.pi + mod2pi(chi_s + np.pi / 2) - mod2pi(ang + ang2)) \
                     + R * mod2pi(2 * np.pi + mod2pi(chi_e - np.pi / 2) - mod2pi(ang + ang2 - np.pi))
            except ValueError:
                L3 = 9999

            # Compute length for case 4 lsl
            ang = atan2(cle.item(1)-cls.item(1), cle.item(0)-cls.item(0))
            L4 = np.linalg.norm(cls-cle) + R * mod2pi(2 * np.pi + mod2pi(chi_s + np.pi / 2) - mod2pi(ang + np.pi / 2)) \
                 + R * mod2pi(2 * np.pi + mod2pi(ang + np.pi / 2) - mod2pi(chi_e + np.pi / 2))

            lengths = [L1, L2, L3, L4]
            if min(lengths) == L1:
                cs = crs
                lam_s = 1
                ce = cre
                lam_e = 1
                q1 = (ce - cs) / np.linalg.norm(ce - cs)
                z1 = cs + R * np.matmul(self.rotz(-np.pi/2), q1)
                z2 = ce + R * np.matmul(self.rotz(-np.pi/2), q1)
                q1 = (z2 - z1) / np.linalg.norm(z2 - z1)

            elif min(lengths) == L2:
                cs = crs
                lam_s = 1
                ce = cle
                lam_e = -1
                l = np.linalg.norm(ce - cs)
                ang = atan2(ce.item(1) - cs.item(1), ce.item(0) - cs.item(0))
                ang2 = ang - np.pi/2 + asin((2 * R) / l)
                q1 = np.matmul(self.rotz(ang2 + np.pi/2), e1)
                z1 = cs + R * np.matmul(self.rotz(ang2), e1)
                z2 = ce + R * np.matmul(self.rotz(ang2 + np.pi), e1)

            elif min(lengths) == L3:
                cs = cls
                lam_s = -1
                ce = cre
                lam_e = 1
                l = np.linalg.norm(ce - cs)
                ang = atan2(ce.item(1) - cs.item(1), ce.item(0) - cs.item(0))
                ang2 = acos((2 * R) / l)
                q1 = np.matmul(self.rotz(ang + ang2 - np.pi/2), e1)
                z1 = cs + R * np.matmul(self.rotz(ang + ang2), e1)
                z2 = ce + R * np.matmul(self.rotz(ang + ang2 -np.pi), e1)

            else:
                cs = cls
                lam_s = -1
                ce = cle
                lam_e = -1
                q1 = (ce - cs) / np.linalg.norm(ce - cs)
                z1 = cs + R * np.matmul(self.rotz(np.pi/2), q1)
                z2 = ce + R * np.matmul(self.rotz(np.pi/2), q1)

            z3 = pe
            q3 = np.matmul(self.rotz(chi_e), e1)

            return [min(lengths), cs, lam_s, ce, lam_e, z1, q1, z2, z3, q3]

        def dot(self, first, second):
            # first and second are np.arrays of size 3
            temp = (first[0]*second[0] + first[1]*second[1] + first[2]*second[2])
            return temp

        def manage_line(self):
            print 'LINE'

            b = self._waypoints[self.index_a]
            a = self.waypoint_temp()

            if (self.index_a == (self._num_waypoints - 1)):
                a = self._waypoints[self.index_a - 1]
            elif (self.index_a == 0):
                a = self._waypoints[self._num_waypoints - 1]
            else:
                a = self._waypoints[self.index_a - 1]


            output = [(a.w0,a.w1),(b.w0,b.w1)]


            if (self.index_a == (self._num_waypoints - 1)):
                self.index_a = 0
            else:
                self.index_a += 1

            return output

        def dubins_points(self, start, end, delta):
            """
            Find points along a dubins path
            :param start: rrtnode for the start point
            :param end: rrtnode for the end point
            :param delta: step size along the path
            :return: list of points
            """
            ps = np.array([[start.w0], [start.w1], [start.w2]])
            pe = np.array([[end.w0], [end.w1], [end.w2]])
            dp_out = self.dubinsParameters(ps, start.chi_d, pe, end.chi_d, self.R_min)

            if dp_out == 0:
                return False

            L, cs, lam_s, ce, lam_e, z1, q1, z2, z3, q3 = dp_out

            points = self.points_along_circle(cs, ps, z1, delta, lam_s)
            points += self.points_along_path(z1, z2, delta)
            points += self.points_along_circle(ce, z2, z3, delta, lam_e)
            return points

        def points_along_path(self, start_pos, end_pos, delta):
            """
            return points along a path separated by a distance delta
            :param start_pos: path start position
            :param end_pos: path end position
            :param delta: distance between each point
            :return: list points along the path
            """
            points = []
            q = end_pos - start_pos
            L = np.linalg.norm(q)
            q = q / L
            for i in range(int(L/delta) + 1):
                cur_pos = start_pos + i * delta * q
                points.append((cur_pos.item(0),cur_pos.item(1)))
            points.append((end_pos.item(0),end_pos.item(1)))
            return points

        def points_along_circle(self, center, start_pos, end_pos, delta, lam):
            """
            Calculates points along a circular arc segment defined by a start, end and center
            position spaced out by a distance of delta
            :param center:
            :param start_pos:
            :param end_pos:
            :param delta:
            :param lambda:
            :return:
            """
            rad = np.linalg.norm(center-start_pos)
            theta_step = delta / rad # will always be positive
            points = []

            start_ang = map_pos_2pi(atan2(start_pos.item(0)-center.item(0), start_pos.item(1)-center.item(1)))
            end_ang = map_pos_2pi(atan2(end_pos.item(0)-center.item(0), end_pos.item(1)-center.item(1)))

            if lam < 0: # counter-clockwise rendering, incrementing theta
                if end_ang - start_ang < 0: # end angle must be greater than start angle
                    start_ang -= 2*pi

                num_steps = int((end_ang-start_ang)/theta_step) # positive

                for i in range(num_steps):
                    x = center.item(0) + rad * sin(start_ang + i * theta_step)
                    y = center.item(1) + rad * cos(start_ang + i * theta_step)
                    points.append(np.array([x, y]))

            if lam > 0: # clockwise rendering, decrementing theta
                if end_ang - start_ang > 0: # end angle must be smaller than start angle
                    end_ang -= 2*pi

                num_steps = int((end_ang-start_ang)/theta_step) # negative

                for i in range(0, num_steps, -1):
                    x = center.item(0) + rad * sin(start_ang + i * theta_step)
                    y = center.item(1) + rad * cos(start_ang + i * theta_step)
                    points.append(np.array([x, y]))

            points.append(np.array([end_pos.item(0), end_pos.item(1), end_pos.item(2)]))
            return points

    # print "start manager"
    manager = path_manager_base(wp_list)

    if len(wp_list) >= 2: # Do NOT iterate unless wp_list has at least 3 waypoints
                          # ...the lower limit may actually be 2--I haven't really checked
                          # the updated algorithms.
        while not manager.index_a == 0:
            manager.iterate()

    return manager.point_list

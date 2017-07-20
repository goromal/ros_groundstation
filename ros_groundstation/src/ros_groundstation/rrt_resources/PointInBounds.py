#!/usr/bin/env python

import numpy as np
from math import *
import rospy
from std_msgs.msg import String
from PointInPolygon import *
import json


class inBounds:

    def __init__(self):
        self.bound_file = "boundaries.txt"

        self.init_lat = 38.144614
        self.init_lon = -76.427506

        self.EARTH_RADIUS = 6370027.0


        self.bounds = []

        with open(self.bound_file) as f:
            for line in f:
                latlon = line.split(' ')
                lat = float(latlon[0])
                lon = float(latlon[1])
                # print lat
                # print lon
                N = self.EARTH_RADIUS*(lat - self.init_lat)*np.pi/180.0;
                E = self.EARTH_RADIUS*cos(lat*np.pi/180.0)*(lon - self.init_lon)*np.pi/180.0;
                p = Point(N, E)
                self.bounds.append(p)

    def pointInBounds(self, p_N, p_E):
        p = Point(p_N, p_E)
        ans = pointInPolygon(p, self.bounds)
        #if not ans:
        #    print "Point out of Bounds"
        return ans

class Obstacle:
    def __init__(self, lat, lon, height, radius):
        self.lat = lat
        self.lon = lon
        self.height = height
        self.radius = radius
        self.N = 0
        self.E = 0
        self.H = 0
        self.rad_m = 0
    def __str__(self):
        return "\nLat:" + str(self.lat) + "\nLon:" + str(self.lon) + "\nHeight:" + str(self.height) + "\nRadius:" + str(self.radius)

class notInObstacle:

    def __init__(self):
        self.init_lat = 38.144614
        self.init_lon = -76.427506
        self.init_h = 6.378
        self.gps_inited = True

        self.EARTH_RADIUS = 6370027.0

        self.inflate_rad = 10 # meters to add to radius

        self.obst_sub = rospy.Subscriber('obstacles', String, self.obst_callback)

        self.obstacles = []
        self.obstacles_init = False

        while not self.gps_inited:
            print "Waiting for gps init"

        while not self.obstacles_init:
            rospy.logwarn_throttle(1,"waiting for obstacles")

        self.obstacles_to_NED()
        print "\nObstacles Ready in NED #:", len(self.obstacles)

    def obstacles_to_NED(self):

        for obst in self.obstacles:
            obst.N = self.EARTH_RADIUS*(obst.lat - self.init_lat)*np.pi/180.0;
            obst.E = self.EARTH_RADIUS*cos(obst.lat*np.pi/180.0)*(obst.lon - self.init_lon)*np.pi/180.0;
            obst.H = obst.height*(0.3048) - self.init_h
            obst.rad_m = obst.radius*(0.3048)
            # print "\nObstacle NED: "
            # print "North:", obst.N
            # print "East:", obst.E
            # print "Rad (m):", obst.rad_m

    def point_not_on_obst(self, N, E):
        for obst in self.obstacles:
            dist = sqrt((float(N) - float(obst.N))**2 + (float(E) - float(obst.E))**2)
            if (dist <= (obst.rad_m + self.inflate_rad)): #and (H <= obst.H + self.inflate_rad):
                print "\nCollide with Obstacle:"
                print "Norths:", obst.N, N
                print "Easts:", obst.E, E
                print "Radius", obst.rad_m
                print "Distance", dist
                return False
        return True

    def obst_callback(self, msg):

        # Read as json dictionary
        self.obstacles = []
        obst_dict = eval(msg.data)
        stat_obst = obst_dict["stationary_obstacles"]
        # mov_obst = obst_dict["moving_obstacles"]

        # Find number of stationary and moving obstacles
        # self.num_stat_obst = len(stat_obst)
        # self.num_mov_obst = len(mov_obst)

        # Init array of obstacle info
        # self.stationary = np.zeros([self.num_stat_obst, 4])
        # self.moving = np.zeros([self.num_mov_obst, 4])


        # Update arrays with ROS msg info
        for _ in range(0, len(stat_obst)):
            lat = stat_obst[_]["latitude"]
            lon = stat_obst[_]["longitude"]
            height = stat_obst[_]["cylinder_height"]
            radius = stat_obst[_]["cylinder_radius"]
            obst = Obstacle(lat, lon, height, radius)
            # print lat, lon, height, radius
            # print "\nObstacle Recieved:", obst
            # Print "Obstacle: ", _, "Recieved"
            self.obstacles.append(obst)

        self.obst_sub.unregister()
        self.obstacles_init = True

        # for _ in range(0, self.num_mov_obst):
        # 	self.moving[_][0] = mov_obst[_]["latitude"]
        # 	self.moving[_][1] = mov_obst[_]["longitude"]
        # 	self.moving[_][2] = mov_obst[_]["altitude_msl"]
        # 	self.moving[_][3] = mov_obst[_]["sphere_radius"]

class pointGood:

    def __init__(self):
        self.bounds_avoid = inBounds()
        self.obst_avoid = notInObstacle()

    def check_point(self, N, E):
        if self.obst_avoid.point_not_on_obst(N, E):
            return self.bounds_avoid.pointInBounds(N, E)
        else:
            return False





# polygon = [Point(0,3), Point(5,3), Point(0,0)]
#
# p = Point(0.5,1)
#
# ans = pointInPolygon(p, polygon)
#
# print ans

# bounders = inBounds()
#
# # print bounders.pointInBounds(0,0)
# rospy.init_node('obst_Avoid')
#
# # obst = notInObstacle()
# #
# # print obst.point_not_on_obst(0, 0, -50)
#
# checker = pointGood()
#
# print checker.check_point(100, 50, 50)
#
# rospy.spin()

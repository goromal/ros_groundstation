import numpy as np
import sys
from math import sin, cos, atan2, sqrt, asin, acos
import random
import rospy
from current_path_generator import get_full_current_path
from PointInBounds import *

def rotz(theta):
    R = np.array([[cos(theta), -sin(theta), 0],
                  [sin(theta), cos(theta), 0],
                  [0, 0, 1]])
    return R

def mod2pi(phi):
    mod = phi%(2 * np.pi)
    return mod

class RRT(object):
    def __init__(self, wpp_start, wpp_end):

        # self.map = area_map
        self.map_shape = 3000 # 1500 in each way (-1500 to 1500) N and E
        self.start_node = rrtNode(0, None, wpp_start)
        self.end_node = rrtNode(0, None, wpp_end)
        self.nodes = set()
        self.nodes.add(self.start_node)
        self.end_nodes = set()
        self.R_min = 50.0
        self.num_valid_paths = 0
        self.path_elevation = self.start_node.pos[2]

        self.checker = pointGood()

    def find_path(self):
        # print "find_path"
        # print "\n start:", self.start_node.pos[0], self.start_node.pos[1], self.start_node.chi
        # print "\n End:", self.end_node.pos[0], self.end_node.pos[1], self.end_node.chi
        # print "Valid Start to End?:", self.check_dubins_path(self.start_node,self.end_node)

        if np.linalg.norm(self.end_node.pos-self.start_node.pos)<5*self.R_min and self.check_dubins_path(self.start_node,self.end_node):
            print "ERROR? WHATS THIS?"
            return [self.convertToFloats(self.start_node), self.convertToFloats(self.end_node)]

        iterations = 0

        while self.num_valid_paths < 10: # find 10 paths before looking for shortest
            # print "try"
            valid_flag = True
            # pick a random point
            x = random.random() * self.map_shape - self.map_shape / 2
            y = random.random() * self.map_shape - self.map_shape / 2
            # print "\nx:", x
            # print "\ny:", y

            # find the nearest node
            nearest = None
            nearest_dist = self.map_shape**3
            chi = 0

            for node in self.nodes:
                dist, ang = node.dist_to(x, y)
                if dist < nearest_dist and dist > 2*self.R_min:
                    nearest_dist = dist
                    nearest = node
                    chi = ang

            if nearest is None:
                # print "no nearest"
                continue

            dist_e, ang_e = self.end_node.dist_to(x,y)
            # print "distance to end: ", dist_e

            if dist_e < 2*self.R_min:
                # print "dist to end < 2 R"
                continue

            # take a step in the right direction
            if dist > 2.2*self.R_min:
                x_test = nearest.pos.item(0) + cos(chi) * 2.2*self.R_min
                y_test = nearest.pos.item(1) + sin(chi) * 2.2*self.R_min
            else:
                x_test = x
                y_test = y
            # print dist
            # print x_test, y_test
            # rospy.logwarn(dist)
            # rospy.logwarn(x)
            # rospy.logwarn(y)
            # rospy.logwarn(chi)
            # rospy.logwarn(dist > 3.2*self.R_min)
            # rospy.logwarn(x_test)
            # rospy.logwarn(y_test)
            pos_test = np.array([[x_test], [y_test], self.path_elevation])
            nearest_dist, chi = nearest.dist_to(x_test,y_test)
            # print "nearest_dist", nearest_dist
            new_node = rrtNode(nearest.cost + nearest_dist, nearest, [pos_test,chi,nearest.Va])

            # get the dubins path between the two
            # print "valid to end?", self.check_dubins_path(new_node, self.end_node)
            valid_flag = self.check_dubins_path(nearest, new_node)
            # print "\n Nearest:", nearest.pos[0], nearest.pos[1]
            # print "\n new_node:", new_node.pos[0], new_node.pos[1]
            # print valid_flag

            # if valid point add a new node
            if valid_flag:
                # add the point to valid nodes stack
                self.nodes.add(new_node)
                # print "#valid nodes =:", len(self.nodes)
                #------------GOOD TO HERE-----------
                # add the path to the map

                # find path to end node
                # dubinsParameters(ps, chi_s, pe, chi_e)
                # check if path to end is valid
                to_end_valid = self.check_dubins_path(new_node, self.end_node)
                # print "\n Nearest:", nearest.pos[0], nearest.pos[1], nearest.chi
                # print "\n new_node:", new_node.pos[0], new_node.pos[1], new_node.chi
                # print "\n end_node", self.end_node.pos[0], self.end_node.pos[1], self.end_node.chi
                # if valid add end node
                if to_end_valid:
                    dist_end, ang_end = new_node.dist_to(self.end_node.pos.item(0), self.end_node.pos.item(1))
                    # return dist_end
                    new_end = rrtNode(new_node.cost + dist_end, new_node, [self.end_node.pos, self.end_node.chi, self.end_node.Va],end_node=True)
                    self.end_nodes.add(new_end)
                    # increment count
                    self.num_valid_paths += 1
                    print "Found valid path #:", self.num_valid_paths
        waypoints = self.find_shortest_path()
        return waypoints


    def check_dubins_path(self, start_node, end_node):
        valid_path = True
        wp1 = np.array([start_node.pos[0][0], start_node.pos[1], start_node.pos[2], start_node.chi]).T
        wp2 = np.array([end_node.pos[0][0], end_node.pos[1], end_node.pos[2], end_node.chi]).T
        path_points = get_full_current_path([wp1, wp2])

        if path_points == False:
            return False

        for point in path_points:
            good = self.checker.check_point(point[0], point[1])

            if not good:
                valid_path = False
                break

        return valid_path



    def find_shortest_path(self):
        node_path = []
        waypoint_path = []
        shortest_node = None
        length = 99999999999999999999
        for node in self.end_nodes:
            if node.cost < length:
                shortest_node = node
                length = node.cost
        node_path.append(shortest_node)
        # rospy.logwarn(shortest_node.cost)
        while node_path[-1].parent is not None:
            node_path.append(node_path[-1].parent)
        node_path.reverse()
        node_path = self.smooth_path(node_path)
        for node in node_path:
            waypoint_path.append(self.convertToFloats(node))
        return waypoint_path

    def smooth_path(self, node_path):
        node_counter = 0
        smooth_path = [node_path[0]]
        end_reached = False
        while not end_reached:
            furthest_possible = node_counter
            for i in range(node_counter+1, len(node_path)):
                if self.check_dubins_path(smooth_path[-1], node_path[i]):
                    furthest_possible = i
            smooth_path.append(node_path[furthest_possible])
            node_counter = furthest_possible
            if smooth_path[-1].end_node:
                end_reached = True
        return smooth_path
    def convertToFloats(self, node):
        return [float(node.pos[0]),float(node.pos[1]),float(node.pos[2]),node.chi,node.Va]

    # def check_path(self, start, end):
    #     valid_path = True
    #     path_points = self.points_along_path(start, end, 10)
    #     for point in path_points:
    #         pt_x = int(point.item(0))
    #         pt_y = int(point.item(1))
    #
    #         if pt_x >= self.map_shape / 2:
    #             pt_x = self.map_shape / 2 - 1
    #         elif pt_x <= -self.map_shape / 2:
    #             pt_x = -self.map_shape / 2 + 1
    #
    #         if pt_y >= self.map_shape / 2:
    #             pt_y = self.map_shape / 2 - 1
    #         elif pt_y <= -self.map_shape / 2:
    #             pt_y = -self.map_shape / 2 + 1
    #
    #         map_x, mapy_y = self.get_map_index(pt_x,pt_y)
    #
    #         if self.map[map_x][map_y] >= -1 * self.path_elevation:
    #             valid_path = False
    #             break
    #     return valid_path

class rrtNode(object):
    """
    Class to hold the nodes for the RRT path planning algorythm
    """
    def __init__(self, cost, parent, waypoint, end_node=False):
        """
        :param cost: path distance to get to the node
        :param parent: previous node in the path chain
        :param waypoint: (x, y, h, angle) the location/course angle of node
                         will contain Va, just ignore it
        """
        self.cost = cost
        self.parent = parent
        self.pos = waypoint[0]
        self.chi = waypoint[1]
        self.Va = waypoint[2]
        self.end_node = end_node

    def dist_to(self, x, y):
        """
        Calculate the distance from the node to an xy location
        elevation is assumed to be the same as the node
        :param x: x location
        :param y: y location
        :return: distance between the node and the location
        """
        node_x = self.pos[0]
        node_y = self.pos[1]
        dist = sqrt((node_x-x)**2 + (node_y-y)**2)
        ang = atan2(y-node_y, x-node_x)
        return dist, ang

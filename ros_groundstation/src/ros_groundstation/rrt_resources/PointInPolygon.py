#!/usr/bin/env python

import numpy as np
from math import *
import rospy

INF = 10000.0

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return "[" + str(self.x) + "," + str(self.y) + "]"

    def __eq__(self, other):
        return (self.x == other.x and self.y == other.y)

def orientation(p1, p2, q1):
    # print q1.x
    # print p2.x
    # print (q1.x - p2.x)
    val = (p2.y - p1.y) * (q1.x - p2.x) - (q1.y - p2.y) * (p2.x - p1.x)
    if (val == 0):
        return 0
    else:
        return np.sign(val)

def onSegment(p1, p2, q):
    if (min(p1.x, p2.x) <= q.x and q.x <= max(p1.x, p2.x)) and \
        (min(p1.y, p2.y) <= q.y and q.y <= max(p1.y, p2.y)):
        return True
    else:
        return False

def intersectionTest(p1, p2, p3, p4):
    # print "p2", p2
    o1 = orientation(p1, p2, p3)
    o2 = orientation(p1, p2, p4)
    o3 = orientation(p3, p4, p1)
    o4 = orientation(p3, p4, p2)

    # General case
    if (o1 != o2 and o3 != o4):
        return True

    # Special Cases
    if (o1 == 0 and onSegment(p1, p2, p3)):
        return True
    if (o2 == 0 and onSegment(p1, p2, p4)):
        return True
    if (o3 == 0 and onSegment(p3, p4, p1)):
        return True
    if (o4 == 0 and onSegment(p3, p4, p2)):
        return True
    # If none of these cases, return False
    return False

def pointInPolygon(p, polygon):
    if len(polygon) < 3:
        print "Polygon has less than 3 sides....."
        return False

    PtoInfinity = Point(INF, p.y)

    intersectionCount = 0
    i = 0
    j = i+1

    ## DO While
    for i in range(0, len(polygon)):
        # print "for"
        # print "poly i", polygon[i]
        # print "poly j", polygon[j]
        if (intersectionTest(p, PtoInfinity, polygon[i], polygon[j]) == True):
            # Weird stuff about point intersecting a joint
            # Just don't let points do that

            invalidIntersection = False
            # if (p.y == polygon[i].y or p.y == polygon[j].y):
            #     res
            if (not invalidIntersection):
                intersectionCount += 1
                # print "Intersection"

                # if (orientation(polygon[i], polygon[j], p) == 0):
                #     if (onSegment(polygon[i], polygon[j], p) == True):
                #         return True
                #     else:
                #         k = (i - 1) % len(polygon) if ((i - 1) >= 0) else (len(polygon) + (i - 1))
                #         w = ((j + 1) % len(polygon))
                #
                #         if ((polygon[k].y <= polygon[i].y and polygon[w].y <= polygon[j].y) \
                #         or (polygon[k].y >= polygon[i].y and polygon[w].y >= polygon[j].y)):
                #             intersectionCount -= 1
        j = ((j + 1) % len(polygon))
    return (intersectionCount % 2 != 0)


# ## Test it here
# point1 = Point(0,0)
# point2 = Point(5,5)
# point3 = Point(10,0)
# # point4 = Point(5,0)
#
# polygon = [point1, point2, point3]
#
#
# # print point3 == point2
# # print point2 == point2
# # print point1
#
# pointx = Point(6,5)
#
# result = pointInPolygon(pointx, polygon)
# print result

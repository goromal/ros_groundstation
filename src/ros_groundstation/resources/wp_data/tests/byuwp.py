#!/usr/bin/python

import math

# Where the plane starts:
byulat = 40.2444580255
byulon = -111.6608

R = 6371000.0
R_prime = math.cos(math.radians(byulat))*R

def dnToLat(dn):
	return byulat + math.degrees(math.asin(dn/R))

def deToLon(de):
	return byulon + math.degrees(math.asin(de/R_prime))

with open('byuwp.txt','r') as infile:
	print('[')
	for line in infile:
		num_pair = line.split(' ')
		lat = dnToLat(float(num_pair[0]))
		lon = deToLon(float(num_pair[1]))
		alt = float(num_pair[2])*3.281
		print("(%f,%f,%f)," % (lat,lon,alt))
	print(']')

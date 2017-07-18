#!/usr/bin/env python

import rospy
from ros_plane.msg import Waypoint
from std_msgs.msg import Float32MultiArray
import math
import sys

# lp1 = sys.argv[1]
# lp2 = sys.argv[2]
# aa = sys.argv[3]

class drop_plan:

	# Init function
	def __init__(self, lat, lon, angle, wind_n, wind_e):
		# self.target_lat = lat      #Target latitude
		# self.target_lon = lon     #Target longitude
		target_global = [lat, lon]
		chi_c = math.radians(angle) # (degrees)
		h_c = 33.0
		Va_c = 15.0
		Vwind_n = wind_n
		Vwind_e = wind_e

		self.angle = angle  #Course angle (in degrees)
		self.alt = 150.0          #desired altitude in feet

		self.init_lat = 0.0
		self.init_lon = 0.0
		self.init_alt = 0.0

		self.target_lat, self.target_lon = self.calc_drop_point(target_global, chi_c, h_c, Va_c, Vwind_n, Vwind_e)
		print "target_lat", self.target_lat
		print "target_lon", self.target_lon

		# # This is the stuff that we need to input just before flight:
		# target_global = [40.2664615, -111.651113]
		# chi_c = math.radians(0.0)    # course angle we want to fly at when we drop the bomb
		# h_c = 36.0                   # drop height entered in meters
		# Va_c = 15.0                  # commanded airspeed in meters
		# Vwind_n = -10                # windspeed in the northern direction (meters)
		# Vwind_e = 5                  # windspeed in the eastern directionm (meters)
		# R_earth = 6370651            # meters

		# Init ROS Node (not when being used with GUI)
		#rospy.init_node('drop_planner', anonymous=True)

		self.wp_pub = rospy.Publisher('/waypoint_path', Waypoint, queue_size=10)

		# subscribe to gps_init
		self.gps_init_sub = rospy.Subscriber('/gps_init', Float32MultiArray, self.gps_init_callback)

		while (self.init_lat == 0.0) and (self.init_lon == 0.0):
			rospy.logwarn("Drop Planner waiting for GPS init")

		self.alt = self.alt*.3048                         #convert altitude in meters

		self.angle = math.radians(angle)               #convert angle in radians

		self.earth_radius = 6370027                  #earth radius in meters

		self.north = self.earth_radius*(self.target_lat - self.init_lat)*math.pi/180.0         #conversion for north
		self.east = self.earth_radius*math.cos(self.init_lat*math.pi/180.0)*(self.target_lon - self.init_lon)*math.pi/180.0    #conversion for east
		self.down = self.init_alt - self.alt      #conversion for down

		self.publishdroppoints()
		rospy.logwarn("BOTTLE DROP WAYPOINTS PUBLISHED")

	def gps_init_callback(self, msg):
		rospy.logwarn("gps_init_callback")
		self.init_lat = msg.data[0]
		self.init_lon = msg.data[1]
		self.init_alt = msg.data[2]
		self.gps_init_sub.unregister()

	def calc_drop_point(self, target_global, chi_c, h_c, Va_c, Vwind_n, Vwind_e):

		# This is the stuff that we need to input just before flight:
		# target_global = [40.2664615, -111.651113]
		# chi_c = math.radians(0.0)    # course angle we want to fly at when we drop the bomb
		# h_c = 36.0                   # drop height entered in meters
		# Va_c = 15.0                  # commanded airspeed in meters
		# Vwind_n = -10                # windspeed in the northern direction (meters)
		# Vwind_e = 5                  # windspeed in the eastern directionm (meters)
		R_earth = 6370027            # meters

		# The rest of this can be left as is...
		#conversions
		conv  = 0.0283495   #convert from oz to kg
		conv2 = 0.0254      #convert from inches to meters
		conv3 = 0.3048      #convert from feet to meters
		#__Water Bottle__
		#physical parameters (down (Z) direction)
		m = 10.0*conv    #mass of the water bottle (output is kg, input oz)
		D = 2.5*conv2   #diameter of bottle in inches (output is meters, input is inches)
		g = 9.81        #gravity (m/s^2)
		rho = 1.225     #density (kg/m^3)
		A_z = 0.25*math.pi*(D**2)   #reference area of the bottle
		Cd_z = 0.5      #drag coefficient measured in wind tunnel
		#__physical parameters (north (X) direction)
		A_x = .00025      #side reference area of bottle (m^2)
		#there were two occasions where A_x and Cd_x were used, and the numbers were different. Which is correct?
		Cd_x = 0.7      #drag coefficient measured in wind tunnel
		#_______END BOTTLE_______
		#___Tuning parameters___
		k_z = (2)*0.5*rho*A_z*Cd_z      #drag constant (fudge factor in parenthesis())
		k_x = (10)*0.5*rho*A_x*Cd_x     #drag constant (fudge factor in parenthesis())
		#___Tuning parameters___
		# calculate approximate Vg at the course angle we want to fly given Va and wind
		Vg_course = math.sqrt((Va_c*math.cos(chi_c) + Vwind_n)**2 + (Va_c*math.sin(chi_c) + Vwind_e)**2)    #approximate ground speed based on Va and windspeed
		#Initial airspeed seen by bottle
		Va0_n = Vg_course*math.cos(chi_c) - Vwind_n
		Va0_e = Vg_course*math.sin(chi_c) - Vwind_e
		#Calculate falling time of the bottle
		t_fall = math.acosh(math.exp(h_c*k_z/m))/math.sqrt(g*k_z/m)       #time for bottle to fall from height h
		dt = 0.001
		t = []
		x = 0.0
		while x <= t_fall:     #discretize t from t = 0 to t = t_fall
		    t.append(x)
		    x += dt
		#Calculate North component of airspeed and ground speed as a function of time for THE BOTTLE.
		Va_n = []
		Vg_n = []
		for i in range(0,len(t)):
		    time = t[i]
		    Va_n.append(Va0_n*(math.exp(-k_x*time/m)))
		    Vg_n.append(Va_n[i] +Vwind_n)
		#Calculate East component of airspeed as a function of time
		Va_e = []
		Vg_e = []
		for i in range(0,len(t)):
		    time = t[i]
		    Va_e.append(Va0_e*(math.exp(-k_x*time/m)))
		    Vg_e.append(Va_e[i] + Vwind_e)
		#Calculate the distance the bottle travels in the north direction
		#initialize x
		x = [0]
		north_final=0
		for i in range(0,len(t)-1):
		    x.append(x[i] + ((Vg_n[i] + Vg_n[i+1])/2)*dt)
		    north_final = north_final + ((Vg_n[i] + Vg_n[i+1])/2)*dt
		#Calculate the distance the bottle travels in the east direction
		#initialize y
		y = [0]
		east_final = 0
		for i in range(0,len(t)-1):
		    y.append(y[i] + ((Vg_e[i] + Vg_e[i+1])/2)*dt)
		    east_final = east_final + ((Vg_e[i] + Vg_e[i+1])/2)*dt
		#Calculate the drop-point relative north and east components
		target = [0,0,0]        #place the target at the home position [n,e,d] = [0,0,0]
		drop_pos_n = target[0] - north_final
		drop_pos_e = target[1] - east_final
		#Generate the actual waypoints in the target-centered local coordinate frame [north_pos, east_pos, height]
		WP_drop = [drop_pos_n, drop_pos_e, h_c]
		print WP_drop
		target_lat = target_global[0]
		target_lon = target_global[1]
		#Calculate the GPS coords of the Drop Point
		lat = target_lat + math.degrees(math.asin(drop_pos_n/R_earth))
		lon = target_lon + math.degrees(math.asin(drop_pos_e/(math.cos(math.radians(target_lat))*R_earth)))
		# print str(lat) + ', ' + str(lon)
		return lat, lon

	def publishdroppoints(self):

		fudge = 25
		self.north -= fudge*math.cos(self.angle)
		self.east -= fudge*math.sin(self.angle)

		comp_north = 50*math.cos(self.angle)
		comp_east = 50*math.sin(self.angle)

		# comp_north2 = 200*math.cos(self.angle)
		# comp_east2 = 200*math.sin(self.angle)

		#WAYPOINT 1
		north1 = self.north - comp_north            #north coordinate of target + north component of 100meters for wypt 1
		east1 = self.east - comp_east                #east coordinate of target + east component of 100meters for wypt 1
		down1 = self.down                            #desired altitude of 120m
		# waypoint1 = [north1, east1, down1, angle]

		#WAYPOINT 2
		north2 = self.north + comp_north            #north coordinate of target - north component of 100meters for wypt 2
		east2 = self.east + comp_east                #east coordinate of target - east component of 100meters for wypt 2
		down2 = self.down
		# waypoint2 = [north2, east2, down2, angle]

		# print(north,east,down)
		# print(waypoint1,waypoint2)

		# Setup drop waypoint
		drop = Waypoint()
		drop.w[0] = self.north
		drop.w[1] = self.east
		drop.w[2] = self.down
		drop.chi_d = self.angle
		drop.Va_d = 15.0
		drop.chi_valid = True # True
		drop.set_current = False
		drop.reset = False
		drop.land = False # True to land now
		drop.drop = True

		# Setup approach 1
		approach1 = Waypoint()
		approach1.w[0] = north1 - 110*math.sin(self.angle)
		approach1.w[1] = east1 - 110*math.cos(self.angle)
		approach1.w[2] = self.down
		approach1.chi_d = self.angle + math.pi
		approach1.Va_d = 15.0
		approach1.chi_valid = True # True
		approach1.set_current = False
		approach1.reset = True
		approach1.land = False
		approach1.drop = False

		# before drop
		b4drop = Waypoint()
		b4drop.w[0] = north1
		b4drop.w[1] = east1
		b4drop.w[2] = self.down
		b4drop.chi_d = self.angle
		b4drop.Va_d = 15.0
		b4drop.chi_valid = True # True
		b4drop.set_current = False
		b4drop.reset = False
		b4drop.land = False # True to land now
		b4drop.drop = False

		# after drop
		aftdrop = Waypoint()
		aftdrop.w[0] = north2
		aftdrop.w[1] = east2
		aftdrop.w[2] = self.down
		aftdrop.chi_d = self.angle
		aftdrop.Va_d = 15.0
		aftdrop.chi_valid = True # True
		aftdrop.set_current = False
		aftdrop.reset = False
		aftdrop.land = False # True to land now
		aftdrop.drop = False

		waypoints = [approach1, b4drop, drop, aftdrop]
		# print land.Va_d, approach1.Va_d, approach3.Va_d

		rospy.sleep(0.5)
	    # Loop through each waypoint
		for wp in waypoints:

			# Publish the Waypoint
			self.wp_pub.publish(wp)

			# Sleep
			d = rospy.Duration(0.05)
			rospy.sleep(d)
		# rospy.signal_shutdown("dropper done")


if __name__ == '__main__':

	# Just run the publisher once
	# def __init__(self, lat, lon, angle, wind_n, wind_e):
	dropper = drop_plan(38.144602, -76.427556, 180.0, 1.0, 0.0)

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

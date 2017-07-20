import rospy
from std_msgs.msg import String, Float32MultiArray #(?)
import json, re
from .Geo import Geobase
from math import fmod, pi

# custom messages
from rosflight_msgs.msg import State, GPS, RCRaw
#from rosplane_msgs.msg import Current_Path, Waypoint #, State?
from ros_plane.msg import Current_Path, Waypoint # FOR TESTING ONLY ---------------

class InitSub(): # could end up being taken from rosplane_msgs.msg: State
    init_latlonalt = [0.0, 0.0, 0.0]
    with_init = False
    enabled = False
    GB = None
    gps_init_topic = None
    gi_sub = None
    @staticmethod
    def updateInitLatLonAlt(new_init_latlonalt):
        print 'taking latlonalt from marble'
        InitSub.reset()
        InitSub.with_init = False
        InitSub.init_latlonalt = new_init_latlonalt
        InitSub.GB = Geobase(InitSub.init_latlonalt[0], InitSub.init_latlonalt[1])
        InitSub.enabled = True

    @staticmethod
    def gi_callback(gps_array):
        InitSub.init_latlonalt[0] = gps_array.data[0]
        InitSub.init_latlonalt[1] = gps_array.data[1]
        InitSub.init_latlonalt[2] = gps_array.data[2]
        InitSub.GB = Geobase(InitSub.init_latlonalt[0], InitSub.init_latlonalt[1])
        InitSub.enabled = True # only perform the calculations if GPS init received
        InitSub.gi_sub.unregister()

    @staticmethod
    def updateGPSInitTopic(new_topic):
        print 'subscribing to', new_topic
        InitSub.reset()
        InitSub.with_init = True
        InitSub.gps_init_topic = new_topic
        InitSub.gi_sub = rospy.Subscriber(InitSub.gps_init_topic, Float32MultiArray, InitSub.gi_callback)

    @staticmethod
    def closeSubscriber():
        InitSub.reset()

    @staticmethod
    def reset():
        InitSub.init_latlonalt = [0.0, 0.0, 0.0]
        InitSub.enabled = False
        InitSub.GB = None
        if not InitSub.gi_sub is None:
            InitSub.gi_sub.unregister()
            InitSub.gi_sub = None

class StateSub():
    state_sub = None
    state_topic = None
    lat = 0.0
    lon = 0.0
    alt = 0.0
    Va = 0.0
    phi = 0.0
    theta = 0.0
    psi = 0.0
    chi = 0.0
    enabled = False

    @staticmethod
    def updateStateTopic(new_state_topic):
        print 'subscribing to', new_state_topic
        StateSub.reset()
        StateSub.state_topic = new_state_topic
        if not StateSub.state_topic is None:
            StateSub.state_sub = rospy.Subscriber(StateSub.state_topic, State, StateSub.state_callback)

    @staticmethod
    def state_callback(state):
        if InitSub.enabled:
            n = state.position[0]
            e = state.position[1]
            d = state.position[2]
            StateSub.lat, StateSub.lon, StateSub.alt = InitSub.GB.ned_to_gps(n, e, d)
            StateSub.alt -= InitSub.init_latlonalt[2]
            StateSub.chi = fmod(state.chi, 2*pi)
            StateSub.Va = state.Va
            StateSub.phi = state.phi
            StateSub.theta = state.theta
            StateSub.psi = state.psi
            StateSub.enabled = True

    @staticmethod
    def closeSubscriber():
        print 'closing subscriber'
        StateSub.reset()

    @staticmethod
    def reset():
        StateSub.enabled = False
        StateSub.lat = 0.0
        StateSub.lon = 0.0
        StateSub.alt = 0.0
        StateSub.Va = 0.0
        StateSub.phi = 0.0
        StateSub.theta = 0.0
        StateSub.psi = 0.0
        StateSub.chi = 0.0
        if not StateSub.state_sub is None:
            StateSub.state_sub.unregister()
            StateSub.state_sub = None

class RCSub():
    rc_sub = None
    rc_raw_topic = None
    autopilotEnabled = True

    @staticmethod
    def updateRCRawTopic(new_rc_raw_topic):
        print 'subscribing to', new_rc_raw_topic
        RCSub.reset()
        RCSub.rc_raw_topic = new_rc_raw_topic
        if not RCSub.rc_raw_topic is None:
            RCSub.rc_sub = rospy.Subscriber(RCSub.rc_raw_topic, RCRaw, RCSub.rc_callback)

    @staticmethod
    def rc_callback(rcRaw):
        RCSub.autopilotEnabled = (rcRaw.values[4] < 1700)

    @staticmethod
    def closeSubscriber():
        print 'closing subscriber'
        RCSub.reset()

    @staticmethod
    def reset():
        RCSub.autopilotEnabled = True
        if not RCSub.rc_sub is None:
            RCSub.rc_sub.unregister()
            RCSub.rc_sub = None

class PathSub():
    path_sub = None
    path_topic = None
    flag = True
    r = [0.0, 0.0, 0.0]
    q = [0.0, 0.0, 0.0]
    c = [0.0, 0.0, 0.0]
    rho = 0.0
    enabled = False

    @staticmethod
    def updatePathTopic(new_path_topic):
        print 'subscribing to', new_path_topic
        PathSub.reset()
        PathSub.path_topic = new_path_topic
        if not PathSub.path_topic is None:
            PathSub.path_sub = rospy.Subscriber(PathSub.path_topic, Current_Path, PathSub.path_callback)

    @staticmethod
    def path_callback(path):
        if InitSub.enabled:
            PathSub.flag = path.flag
            r_lat, r_lon, r_alt = InitSub.GB.ned_to_gps(path.r[0], path.r[1], path.r[2])
            PathSub.r = [r_lat, r_lon, r_alt]
            PathSub.q = [path.q[0], path.q[1], path.q[2]]
            c_lat, c_lon, c_alt = InitSub.GB.ned_to_gps(path.c[0], path.c[1], path.c[2])
            PathSub.c = [c_lat, c_lon, c_alt]
            PathSub.rho = path.rho
            PathSub.enabled = True

    @staticmethod
    def closeSubscriber():
        print 'closing subscriber'
        PathSub.reset()

    @staticmethod
    def reset():
        PathSub.enabled = False
        PathSub.flag = True
        PathSub.r = [0.0, 0.0, 0.0]
        PathSub.q = [0.0, 0.0, 0.0]
        PathSub.c = [0.0, 0.0, 0.0]
        PathSub.rho = 0.0
        if not PathSub.path_sub is None:
            PathSub.path_sub.unregister()
            PathSub.path_sub = None

class renderable_wp():
    def __init__(self, lat, lon, alt, chi_d, chi_valid, Va_d, converted=True):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.chi_d = chi_d # radians
        self.chi_valid = chi_valid
        self.Va_d = Va_d
        self.converted = converted

class WaypointSub():
    wp_sub = None
    waypoint_topic = None
    waypoints = []
    enabled = False

    @staticmethod
    def updateWaypointTopic(new_waypoint_topic):
        print 'subscribing to', new_waypoint_topic
        WaypointSub.reset()
        WaypointSub.waypoint_topic = new_waypoint_topic
        if not WaypointSub.waypoint_topic is None:
            WaypointSub.wp_sub = rospy.Subscriber(WaypointSub.waypoint_topic, Waypoint, WaypointSub.waypoint_callback)

    @staticmethod
    def waypoint_callback(wp):
        if InitSub.enabled:
            lat, lon, alt = InitSub.GB.ned_to_gps(wp.w[0], wp.w[1], wp.w[2])
            WaypointSub.waypoints.append(renderable_wp(lat, lon, alt, wp.chi_d, wp.chi_valid, wp.Va_d))
            for rwp in WaypointSub.waypoints:
                if not rwp.converted:
                    rwp.lat, rwp.lon, rwp.alt = InitSub.GB.ned_to_gps(rwp.lat, rwp.lon, rwp.alt)
                    rwp.converted = True
            WaypointSub.enabled = True
        else:
            WaypointSub.waypoints.append(renderable_wp(wp.w[0], wp.w[1], wp.w[2], wp.chi_d, wp.chi_valid, wp.Va_d, False))
            WaypointSub.enabled = False

    @staticmethod
    def closeSubscriber():
        print 'closing subscriber'
        WaypointSub.reset()

    @staticmethod
    def reset():
        WaypointSub.enabled = False
        WaypointSub.waypoints = []
        if not WaypointSub.wp_sub is None:
            WaypointSub.wp_sub.unregister()
            WaypointSub.wp_sub = None

class ObstacleSub():
    obs_sub = None
    obstacle_topic = None
    stationaryObstacles = []
    movingObstacles = []
    enabled = False

    @staticmethod
    def updateObstacleTopic(new_obstacle_topic):
        print 'subscribing to', new_obstacle_topic
        ObstacleSub.reset()
        ObstacleSub.obstacle_topic = new_obstacle_topic
        if not ObstacleSub.obstacle_topic is None:
            ObstacleSub.obs_sub = rospy.Subscriber(ObstacleSub.obstacle_topic, String, ObstacleSub.json_callback)

    @staticmethod
    def json_callback(obstacles_json):
        json_data = str(obstacles_json.data)
        json_data = re.sub(r"u'",r'"',json_data)
        json_data = re.sub(r"'",r'"',json_data)
        data = json.loads(json_data)
        moving_obstacles = data["moving_obstacles"]
        stationary_obstacles = data["stationary_obstacles"]

        ObstacleSub.movingObstacles = []
        for obstacle in moving_obstacles:
            lat = float(obstacle["latitude"])
            lon = float(obstacle["longitude"])
            radius = float(obstacle["sphere_radius"])
            height = float(obstacle["altitude_msl"])
            ObstacleSub.movingObstacles.append((lat, lon, radius, height))

        ObstacleSub.stationaryObstacles = []
        for obstacle in stationary_obstacles:
            lat = float(obstacle["latitude"])
            lon = float(obstacle["longitude"])
            radius = float(obstacle["cylinder_radius"])
            height = float(obstacle["cylinder_height"])
            ObstacleSub.stationaryObstacles.append((lat, lon, radius, height))
        ObstacleSub.enabled = True

    @staticmethod
    def closeSubscriber():
        print 'closing subscriber'
        ObstacleSub.reset()

    @staticmethod
    def reset():
        ObstacleSub.enabled = False
        ObstacleSub.stationaryObstacles = []
        ObstacleSub.movingObstacles = []
        if not ObstacleSub.obs_sub is None:
            ObstacleSub.obs_sub.unregister()
            ObstacleSub.obs_sub = None

class GPSDataSub():
    gps_sub = None
    gps_data_topic = None
    numSat = 0
    enabled = False

    @staticmethod
    def updateGPSDataTopic(new_gps_data_topic):
        print 'subscribing to', new_gps_data_topic
        GPSDataSub.reset()
        GPSDataSub.gps_data_topic = new_gps_data_topic
        if not GPSDataSub.gps_data_topic is None:
            GPSDataSub.gps_sub = rospy.Subscriber(GPSDataSub.gps_data_topic, GPS, GPSDataSub.callback_GPS)

    @staticmethod
    def callback_GPS(gps_data):
        GPSDataSub.numSat = gps_data.NumSat
        GPSDataSub.enabled = True

    @staticmethod
    def closeSubscriber():
        print 'closing subscriber'
        GPSDataSub.reset()

    @staticmethod
    def reset():
        GPSDataSub.enabled = False
        GPSDataSub.numSat = 0
        if not GPSDataSub.gps_sub is None:
            GPSDataSub.gps_sub.unregister()
            GPSDataSub.gps_sub = None

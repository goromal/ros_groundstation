import rospy
from std_msgs.msg import String
import json, re
from .Geo import Geobase
from math import fmod, pi

# custom messages
from rosflight_msgs.msg import RCRaw
from inertial_sense.msg import GPS
from rosplane_msgs.msg import Current_Path, Waypoint, State, Controller_Internals, Controller_Commands
from uav_msgs.msg import JudgeMission, NED_list, NED_pt, Point, OrderedPoint
from uav_msgs.srv import GetMissionWithId, PlanMissionPoints, UploadPath

class InitSub():
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
    def state_callback(state):
        InitSub.init_latlonalt[0] = state.latitude
        InitSub.init_latlonalt[1] = state.longitude
        InitSub.init_latlonalt[2] = state.altitude
        InitSub.GB = Geobase(InitSub.init_latlonalt[0], InitSub.init_latlonalt[1])
        InitSub.enabled = True # only perform the calculations if GPS init received
        InitSub.gi_sub.unregister()

    @staticmethod
    def updateGPSInitTopic(new_topic):
        print 'subscribing to', new_topic
        InitSub.reset()
        InitSub.with_init = True
        InitSub.gps_init_topic = new_topic
        InitSub.gi_sub = rospy.Subscriber(InitSub.gps_init_topic, GPS, InitSub.state_callback)

    @staticmethod
    def getGPSInitTopic():
        return InitSub.gps_init_topic

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

class MissionSub():
    enabled = False
    boundaries = []
    obstacles = []
    waypoints = []
    mission_proxy = rospy.ServiceProxy('get_mission_with_id', GetMissionWithId)
    @staticmethod
    def getMission():
        MissionSub.enabled = False
        MissionSub.boundaries = []
        MissionSub.waypoints = []
        MissionSub.obstacles = []
        # try:
        #     response = MissionSub.mission_proxy(0)
        #     for waypoint in response.mission.waypoints:
        #         lat = boundary.point.latitude
        #         lon = boundary.point.longitude
        #         MissionSub.waypoints.append([lat, lon])
        #     for boundary in response.mission.boundaries:
        #         lat = boundary.point.latitude
        #         lon = boundary.point.longitude
        #         MissionSub.boundaries.append([lat, lon])
        #     MissionSub.boundaries.append(MissionSub.boundaries[0])
        #     for obstacle in response.mission.stationary_obstacles:
        #         lat = obstacle.point.latitude
        #         lon = obstacle.point.longitude
        #         rad = obstacle.cylinder_radius
        #         N, E, D = InitSub.GB.gps_to_ned(lat, lon)
        #         lat_ul, lon_ul, alt_ul = InitSub.GB.ned_to_gps(N+rad,E-rad,D)
        #         lat_lr, lon_lr, alt_lr = InitSub.GB.ned_to_gps(N-rad,E+rad,D)
        #         MissionSub.obstacles.append([lat_ul, lon_ul, lat_lr, lon_lr])
        #     MissionSub.enabled = True
        # except:
        #     return
        response = MissionSub.mission_proxy(0)
        for waypoint in response.mission.waypoints:
            lat = waypoint.point.latitude
            lon = waypoint.point.longitude
            MissionSub.waypoints.append([lat, lon])
        for boundary in response.mission.boundaries:
            lat = boundary.point.latitude
            lon = boundary.point.longitude
            MissionSub.boundaries.append([lat, lon])
        MissionSub.boundaries.append(MissionSub.boundaries[0])
        for obstacle in response.mission.stationary_obstacles:
            lat = obstacle.point.latitude
            lon = obstacle.point.longitude
            rad = obstacle.cylinder_radius
            N, E, D = InitSub.GB.gps_to_ned(lat, lon)
            lat_ul, lon_ul, alt_ul = InitSub.GB.ned_to_gps(N+rad,E-rad,D)
            lat_lr, lon_lr, alt_lr = InitSub.GB.ned_to_gps(N-rad,E+rad,D)
            MissionSub.obstacles.append([lat_ul, lon_ul, lat_lr, lon_lr])
        MissionSub.enabled = True

# "POINTS AND PATHS" Subscriber
class PPSub():
    enabled = False
    land_wps = [[],[]]
    path_wps = []
    approved = False
    mission_type = 0
    approval_proxy = rospy.ServiceProxy('approved_path', UploadPath)
    path_wps_proxy = rospy.ServiceProxy('plan_path', PlanMissionPoints)

    @staticmethod
    def changeMissionType(type):
        # PPSub.enabled = False
        PPSub.approved = False
        PPSub.path_wps = []
        PPSub.mission_type = type

    @staticmethod
    def setFirstLandingWaypoint(waypoint):
        PPSub.land_wps[0] = waypoint
        PPSub.enabled = True

    @staticmethod
    def setSecondLandingWaypoint(waypoint):
        PPSub.land_wps[1] = waypoint
        PPSub.enabled = True

    @staticmethod
    def resetLandingWaypoints():
        PPSub.land_wps = [[],[]]

    @staticmethod
    def getPath():
        PPSub.enabled = False
        PPSub.approved = False
        PPSub.path_wps = []
        try:
            if PPSub.mission_type == 4 and len(PPSub.land_wps[0]) > 0 and len(PPSub.land_wps[1]) > 0:
                wp1 = NED_pt()
                wp1.N, wp1.E, wp1.D = InitSub.GB.gps_to_ned(PPSub.land_wps[0][0], PPSub.land_wps[0][1], PPSub.land_wps[0][2])
                wp2 = NED_pt()
                wp2.N, wp2.E, wp2.D = InitSub.GB.gps_to_ned(PPSub.land_wps[1][0], PPSub.land_wps[1][1], PPSub.land_wps[1][2])
                landingList = NED_list()
                landingList.waypoint_list.append(wp1)
                landingList.waypoint_list.append(wp2)
                response = PPSub.path_wps_proxy(PPSub.mission_type, landingList)
            else:
                response = PPSub.path_wps_proxy(PPSub.mission_type, NED_list())
            for NED in response.planned_waypoints.waypoint_list:
                lat, lon, alt = InitSub.GB.ned_to_gps(NED.N, NED.E, NED.D)
                PPSub.path_wps.append([lat, lon])
            PPSub.enabled = True
        except:
            return

    @staticmethod
    def approvePath():
        try:
            PPSub.approved = PPSub.approval_proxy()
        except:
            return

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
    def getStateTopic():
        return StateSub.state_topic

    @staticmethod
    def state_callback(state):
        if InitSub.enabled:
            n = state.position[0]
            e = state.position[1]
            d = state.position[2]
            StateSub.lat, StateSub.lon, StateSub.alt = InitSub.GB.ned_to_gps(n, e, d)
            #StateSub.alt -= InitSub.init_latlonalt[2]
            StateSub.chi = fmod(state.chi, 2*pi)
            StateSub.Va = state.Va
            StateSub.phi = state.phi
            StateSub.theta = state.theta
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
    channel = 6

    @staticmethod
    def updateRCRawTopic(new_rc_raw_topic):
        print 'subscribing to', new_rc_raw_topic
        RCSub.reset()
        RCSub.rc_raw_topic = new_rc_raw_topic
        if not RCSub.rc_raw_topic is None:
            RCSub.rc_sub = rospy.Subscriber(RCSub.rc_raw_topic, RCRaw, RCSub.rc_callback)

    @staticmethod
    def getRCRawTopic():
        return RCSub.rc_raw_topic

    @staticmethod
    def updateRCChannel(new_rc_channel):
        print 'updating RC channel to', new_rc_channel
        RCSub.channel = new_rc_channel

    @staticmethod
    def rc_callback(rcRaw):
        RCSub.autopilotEnabled = (rcRaw.values[RCSub.channel] < 950) # <<<<<

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
            RCSub.channel = 5

class PathSub():
    path_sub = None
    path_topic = None
    path_type = 0
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
    def getPathTopic():
        return PathSub.path_topic

    @staticmethod
    def path_callback(path):
        if InitSub.enabled:
            PathSub.path_type = path.path_type
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
        PathSub.path_type = 0
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
    def getWaypointTopic():
        return WaypointSub.waypoint_topic

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
    def getObstacleTopic():
        return ObstacleSub.obstacle_topic

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
    def getGPSDataTopic():
        return GPSDataSub.gps_data_topic

    @staticmethod
    def callback_GPS(gps_data):
        GPSDataSub.numSat = gps_data.num_sat
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

class ConInSub():
    con_in_sub = None
    controller_inners_topic = None
    theta_c = 0.0
    phi_c = 0.0
    enabled = False

    @staticmethod
    def updateConInTopic(new_controller_inners_topic):
        print 'subscribing to', new_controller_inners_topic
        ConInSub.reset()
        ConInSub.controller_inners_topic = new_controller_inners_topic
        if not ConInSub.controller_inners_topic is None:
            ConInSub.con_in_sub = rospy.Subscriber(ConInSub.controller_inners_topic, Controller_Internals, ConInSub.callback_ConIn)

    @staticmethod
    def getConInTopic():
        return ConInSub.controller_inners_topic

    @staticmethod
    def callback_ConIn(controller_internals):
        ConInSub.theta_c = controller_internals.theta_c
        ConInSub.phi_c = controller_internals.phi_c
        ConInSub.enabled = True

    @staticmethod
    def closeSubscriber():
        print 'closing subscriber'
        ConInSub.reset()

    @staticmethod
    def reset():
        ConInSub.enabled = False
        ConInSub.theta_c = 0.0
        ConInSub.phi_c = 0.0
        if not ConInSub.con_in_sub is None:
            ConInSub.con_in_sub.unregister()
            ConInSub.con_in_sub = None

class ConComSub():
    con_com_sub = None
    controller_commands_topic = None
    Va_c = 0.0
    h_c = 0.0
    chi_c = 0.0
    enabled = False

    @staticmethod
    def updateConComTopic(new_controller_commands_topic):
        print 'subscribing to', new_controller_commands_topic
        ConComSub.reset()
        ConComSub.controller_commands_topic = new_controller_commands_topic
        if not ConComSub.controller_commands_topic is None:
            ConComSub.con_com_sub = rospy.Subscriber(ConComSub.controller_commands_topic, Controller_Commands, ConComSub.callback_ConCom)

    @staticmethod
    def getConComTopic():
        return ConComSub.controller_commands_topic

    @staticmethod
    def callback_ConCom(controller_commands):
        ConComSub.Va_c = controller_commands.Va_c
        ConComSub.h_c = controller_commands.h_c
        ConComSub.chi_c = controller_commands.chi_c
        ConComSub.enabled = True

    @staticmethod
    def closeSubscriber():
        print 'closing subscriber'
        ConComSub.reset()

    @staticmethod
    def reset():
        ConComSub.enabled = False
        ConComSub.Va_c = 0.0
        ConComSub.h_c = 0.0
        ConComSub.chi_c = 0.0
        if not ConComSub.con_com_sub is None:
            ConComSub.con_com_sub.unregister()
            ConComSub.con_com_sub = None

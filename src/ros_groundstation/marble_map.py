from .gm_plotter import GoogleMapPlotter
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import os.path
from math import sin, cos, radians

import map_info_parser
from Signals import WP_Handler
from .Geo import Geobase #--??--
from .map_subscribers import *

class MarbleMap(QWidget):
    def __init__(self, parent=None):
        super(MarbleMap, self).__init__() # QWidget constructor
        self.WPH = WP_Handler()
        self.wp_opts = {'None':{'title_substr':'Empty Mode','folder_name':'','is_rrt':False},
                        'MainWP':{'title_substr':'Main Waypoints','folder_name':'main_wps','is_rrt':False},
                        'PathWP':{'title_substr':'Path Waypoints','folder_name':'path_wps','is_rrt':False},
                        'SearchWP':{'title_substr':'Search Waypoints','folder_name':'search_wps','is_rrt':False},
                        'RRT_PathWP':{'title_substr':'RRT Path Waypoints','folder_name':'rrt_path_wps','is_rrt':True,'parent':'PathWP','needs_render':True},
                        'RRT_SearchWP':{'title_substr':'RRT Search Waypoints','folder_name':'rrt_search_wps','is_rrt':True,'parent':'SearchWP','needs_render':True},
                        'DropWP':{'title_substr':'Bottle Drop Waypoints','folder_name':'drop_wps','is_rrt':False},
                        'HikerWP':{'title_substr':'Hiker Waypoints','folder_name':'hiker_wps','is_rrt':False}}
        self.wp_state = 'None' # can be 'None','MainWP','PathWP','SearchWP',
                               #        'DropWP','TargetWP','HikerWP'
        # For waypoint conversion
        self._home_map = map_info_parser.get_default()

        self._gps_dict = map_info_parser.get_gps_dict()

        self.latlon = self._gps_dict[self._home_map][0]
        self.latlonalt = [self.latlon[0], self.latlon[1], 0.0] # FOR INTERFACING WITH SUBSCRIBERS
        self.zoom = self._gps_dict[self._home_map][1]
        self.get_size()
        self.GMP = GoogleMapPlotter(self._gps_dict, self.w_width, self.w_height, self._home_map)

        self.update() # is this all I need, theoretically? or do I need to overload it?

        self.GB = Geobase(self.latlon[0], self.latlon[1]) # For full current path drawer
        self._mouse_attentive = False
        self.movement_offset = QPoint(0,0)

        self.seconds_tests = [2.0/3600, 5.0/3600, 15.0/3600, 30.0/3600, 60.0/3600]
        self.num_s_tests = len(self.seconds_tests)

        self.view_full_path = False
        self.current_path_NE_list = []

        self.mouse_event_counter = 0
        self.counter_limit = 4
        self.lon_multiplier = -0.25
        self.lat_multiplier = -0.25
        self.wheel_angle = 0
        self.wheel_angle_thresh = 1*120

        # geometric items for drawing
        self.plane_h = 30 # pixels
        self.plane_w = 25 # pixels

    def get_size(self):
        frame_size = self.frameSize()
        self.w_width = frame_size.width()
        self.w_height = frame_size.height()

    def resizeEvent(self, QResizeEvent):
        self.get_size()
        self.GMP.UpdateSize(self.w_width, self.w_height)

    def wheelEvent(self, QWheelEvent):
        self.wheel_angle += QWheelEvent.angleDelta().y()
        if self.wheel_angle >= self.wheel_angle_thresh:
            self.GMP.UpdateZoom(1)
            self.wheel_angle = 0
        if self.wheel_angle <= -self.wheel_angle_thresh:
            self.GMP.UpdateZoom(-1)
            self.wheel_angle = 0

    def mouseMoveEvent(self, QMouseEvent):
        if not self._mouse_attentive: # we won't do anything with movement in point-and-click mode
            if QMouseEvent.buttons(): # in act of dragging
                self.mouse_event_counter += 1
                if self.mouse_event_counter > self.counter_limit:
                    qpoint_delta = QMouseEvent.pos() - self.movement_offset
                    delta_x = self.lon_multiplier * qpoint_delta.x()
                    lon_incremented = GoogleMapPlotter.pix_to_rel_lon(self.GMP.center.lon, delta_x, self.GMP.zoom)
                    delta_y = self.lat_multiplier * qpoint_delta.y()
                    lat_incremented = GoogleMapPlotter.pix_to_rel_lat(self.GMP.center.lat, delta_y, self.GMP.zoom)
                    self.GMP.UpdateView(lat_incremented, lon_incremented)
                    self.mouse_event_counter = 0

    def recenter(self):
        self.GMP.UpdateView(self.latlon[0], self.latlon[1])

    def enterEvent(self, QEvent):
        if self._mouse_attentive:
            self.setCursor(QCursor(Qt.CrossCursor))
        else:
            self.setCursor(QCursor(Qt.OpenHandCursor))

    def leaveEvent(self, QEvent):
        self.setCursor(QCursor(Qt.ArrowCursor))

    def mousePressEvent(self, QMouseEvent):
        if self._mouse_attentive:
            print 'OUCH'
            #self.WPH.emit_clicked(clicked_lat, clicked_lon)
        else:
            self.movement_offset = QMouseEvent.pos()
            self.setCursor(QCursor(Qt.ClosedHandCursor))

    def mouseReleaseEvent(self, QMouseEvent):
        if not self._mouse_attentive:
            self.setCursor(QCursor(Qt.OpenHandCursor))

    def path_viewer_toggle(self, state_integer):
        if state_integer == 2: # checkbox checked
            self.view_full_path = True
        else: # checkbox unchecked
            self.view_full_path = False

    def change_home(self, map_name):
        self._home_map = map_name
        self.latlon = self._gps_dict[self._home_map][0]
        self.latlonalt = [self.latlon[0], self.latlon[1], 0.0] # FOR INTERFACING WITH SUBSCRIBERS
        if not InitSub.with_init:
            InitSub.updateInitLatLonAlt(self.latlonalt)
        self.zoom = self._gps_dict[self._home_map][1]
        self.GB = Geobase(self.latlon[0], self.latlon[1])
        self.GMP.UpdateMap(map_name)
        self.update()
        self.WPH.emit_home_change(self._home_map)

    # =====================================================
    # ==================== FOR DRAWING ====================
    # =====================================================

    def paintEvent(self, QPaintEvent):
        painter = QPainter()
        painter.begin(self)
        painter.setRenderHint(QPainter.Antialiasing, True)

        upper_left = QPoint(0, 0)
        painter.drawImage(upper_left, self.GMP.GetImage())

        # Draw center crosshairs (probably temporary until smartzoom feature is implemented)
        painter.setPen(QPen(QBrush(Qt.blue), 2, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(self.GMP.width/2, self.GMP.height/2-8, self.GMP.width/2, self.GMP.height/2+8)
        painter.drawLine(self.GMP.width/2-8, self.GMP.height/2, self.GMP.width/2+8, self.GMP.height/2)

        if WaypointSub.enabled:
            self.draw_waypoints(painter)
        if ObstacleSub.enabled:
            self.draw_obstacles(painter)
        if PathSub.enabled:
            self.draw_currentpath(painter)
        if StateSub.enabled:
            self.draw_plane(painter)

        painter.end()

    # paint using self.GMP.center and self.GMP.lat/lon_to_pix_coefficient
    def draw_waypoints(self, painter):
        painter.setPen(QPen(QBrush(Qt.darkRed), 2.5, Qt.SolidLine, Qt.RoundCap))
        # it can be assumed that all waypoints are converted to latlon if the sub is enabled
        for waypoint in WaypointSub.waypoints:
            x = self.lon_to_pix(waypoint.lon)
            y = self.lat_to_pix(waypoint.lat)
            if x >=0 and x <= self.GMP.width and y >= 0 and y <= self.GMP.height:
                rad = 5
                painter.drawEllipse(x-rad, y-rad, 2*rad, 2*rad)
                if waypoint.chi_valid:
                    painter.drawLine(x, y, x+2*rad*sin(waypoint.chi_d), y-2*rad*cos(waypoint.chi_d))

    def draw_obstacles(self, painter):
        pass # ++++++++++++++++++++++++++++++++++++++++++

    def draw_currentpath(self, painter):
        painter.setPen(QPen(QBrush(Qt.red), 3.5, Qt.SolidLine, Qt.RoundCap))
        if PathSub.flag == True:
            r = PathSub.r # [lat, lon]
            q = PathSub.q # unit length, NED
            scale = 200   # pixels
            pt_1 = [self.lon_to_pix(r[1]), self.lat_to_pix(r[0])]
            if pt_1[0] >=0 and pt_1[0] <= self.GMP.width and pt_1[1] >= 0 and pt_1[1] <= self.GMP.height:
                pt_2 = [pt_1[0]+scale*q[1],pt_1[1]-scale*q[0]]
                painter.drawLine(pt_1[0], pt_1[1], pt_2[0], pt_2[1])
        else:
            c = PathSub.c # [lat, lon]
            R = PathSub.rho # meters
            pt_c = [self.lon_to_pix(c[1]), self.lat_to_pix(c[0])]
            if pt_c[0] >=0 and pt_c[0] <= self.GMP.width and pt_c[1] >= 0 and pt_c[1] <= self.GMP.height:
                R_pix = R * 2**self.GMP.zoom / (156543.03392 * cos(radians(c[0])))
                painter.drawEllipse(pt_c[0]-R_pix, pt_c[1]-R_pix, 2*R_pix, 2*R_pix)

    def draw_plane(self, painter):
        if RCSub.autopilotEnabled:
            painter.setPen(QPen(QBrush(Qt.red), 5, Qt.SolidLine, Qt.RoundCap))
        else:
            painter.setPen(QPen(QBrush(Qt.cyan), 5, Qt.SolidLine, Qt.RoundCap))

        x = self.lon_to_pix(StateSub.lon)
        y = self.lat_to_pix(StateSub.lat)

        if x >=0 and x <= self.GMP.width and y >= 0 and y <= self.GMP.height:
            #print 'plane at', x, y
            chi = StateSub.chi

            pt_1_x = x + self.rotate_x(0, self.plane_h/2, chi)
            pt_1_y = y - self.rotate_y(0, self.plane_h/2, chi)
            pt_2_x = x + self.rotate_x(0, -self.plane_h/2, chi)
            pt_2_y = y - self.rotate_y(0, -self.plane_h/2, chi)
            pt_3_x = x
            pt_3_y = y
            pt_4_x = x + self.rotate_x(-self.plane_w/2, -self.plane_h/4, chi)
            pt_4_y = y - self.rotate_y(-self.plane_w/2, -self.plane_h/4, chi)
            pt_5_x = x + self.rotate_x(self.plane_w/2, -self.plane_h/4, chi)
            pt_5_y = y - self.rotate_y(self.plane_w/2, -self.plane_h/4, chi)
            pt_6_x = x + self.rotate_x(-self.plane_w/4, -2*self.plane_h/5, chi)
            pt_6_y = y - self.rotate_y(-self.plane_w/4, -2*self.plane_h/5, chi)
            pt_7_x = x + self.rotate_x(self.plane_w/4, -2*self.plane_h/5, chi)
            pt_7_y = y - self.rotate_y(self.plane_w/4, -2*self.plane_h/5, chi)

            painter.drawLine(pt_1_x, pt_1_y, pt_2_x, pt_2_y)
            painter.drawLine(pt_3_x, pt_3_y, pt_4_x, pt_4_y)
            painter.drawLine(pt_3_x, pt_3_y, pt_5_x, pt_5_y)
            painter.drawLine(pt_6_x, pt_6_y, pt_7_x, pt_7_y)
            painter.setPen(QPen(QBrush(Qt.black), 2.5, Qt.SolidLine, Qt.RoundCap))
            painter.drawLine(pt_1_x, pt_1_y, pt_2_x, pt_2_y)
            painter.drawLine(pt_3_x, pt_3_y, pt_4_x, pt_4_y)
            painter.drawLine(pt_3_x, pt_3_y, pt_5_x, pt_5_y)
            painter.drawLine(pt_6_x, pt_6_y, pt_7_x, pt_7_y)

    def lon_to_pix(self, lon): # assuming origin at upper left
        return GoogleMapPlotter.rel_lon_to_rel_pix(self.GMP.west, lon, self.GMP.zoom)

    def lat_to_pix(self, lat): # assuming origin at upper left
        return GoogleMapPlotter.rel_lat_to_rel_pix(self.GMP.north, lat, self.GMP.zoom)

    def rotate_x(self, x, y, a):
        return x * cos(a) + y * sin(a)

    def rotate_y(self, x, y, a):
        return -1 * x * sin(a) + y * cos(a)

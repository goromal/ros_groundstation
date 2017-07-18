from python_qt_binding import loadUi
from PyQt5.Qt import *
from current_path_generator import get_full_current_path
from drop_planner import drop_plan
from rrt_resources.new_rrt_path_planner import write_waypoints

QString = type("")

import os, rospy, map_info_parser
from ros_plane.msg import Waypoint

PWD = os.path.dirname(os.path.abspath(__file__))

class WP_Publisher():
    def __init__(self):
        self.pub = rospy.Publisher('/waypoint_path', Waypoint, queue_size=50)

    def publish_wp_to_plane(self, wp, chi, flag, land):
        wp_obj = Waypoint()
        wp_obj.w[0] = wp[0]
        wp_obj.w[1] = wp[1]
        wp_obj.w[2] = wp[2]
        wp_obj.chi_d = chi # course for this waypoint
        wp_obj.chi_valid = True # determines if dubins is used
                                 # (see ros_plane -> path_manager_example.cpp)
        wp_obj.reset = flag # True for the first waypoint sent
        wp_obj.land = land  # True if in landing mode
        wp_obj.Va_d = 15.0 # m/s
        wp_obj.set_current = False # sets to be executed now
        wp_obj.drop = False
        self.pub.publish(wp_obj)

class WpWindow(QWidget):
    def __init__(self, marble, uifname = 'wp_window.ui'):
        super(WpWindow, self).__init__()
        self.marble = marble
        ui_file = os.path.join(PWD, 'resources', uifname)
        loadUi(ui_file, self)
        self.setObjectName(uifname)

        # Set up waypoints and waypoint files
        self._home_map = self.marble._home_map
        self.WPP = WP_Publisher()
        self.load_wp_from_file()
        self.update_lists()
        self.set_title()

        # Set up event triggers
        self.send_to_plane_button.clicked.connect(self.transfer_waypoint_data)
        self.mode_comboBox.clear()
        self.mode_comboBox.addItem(QString('Empty Mode'))
        self.mode_comboBox.addItem(QString('Main Mode'))
        self.mode_comboBox.addItem(QString('Path Mode'))
        self.mode_comboBox.addItem(QString('RRT Path Mode'))
        self.mode_comboBox.addItem(QString('Search Mode'))
        self.mode_comboBox.addItem(QString('RRT Search Mode'))
        self.mode_comboBox.addItem(QString('Drop Mode'))
        self.mode_comboBox.addItem(QString('Hiker Mode'))
        self.mode_texts = {'Empty Mode':'None','Main Mode':'MainWP',
                           'Path Mode':'PathWP','RRT Path Mode':'RRT_PathWP',
                           'Search Mode':'SearchWP','RRT Search Mode':'RRT_SearchWP',
                           'Drop Mode':'DropWP','Hiker Mode':'HikerWP'}
        self.mode_comboBox.currentIndexChanged.connect(self.load_wp_mode)

        # For signal handling
        self.marble.WPH.home_changed.connect(self.change_home)

        # For bottle drop
        self.Vw_comps = map_info_parser.get_windspeed_components() # [Vwind_n, Vwind_e]

        #self.rendered_rrt_path = False
        #self.rendered_rrt_search = False
    # CHANGE TRIGGERS

    def load_wp_mode(self):
        self.marble.wp_state = self.mode_texts[self.mode_comboBox.currentText()]
        self.full_update()

    def change_home(self, new_home):
        self._home_map = new_home
        self.full_update()

    # UPDATE HANDLERS

    def full_update(self):
        self.NED_waypoints = []
        for i in range(len(self.waypoints)): # clear map waypoints
            self.marble.WPH.emit_removed(0)
        self.load_wp_from_file() # update self.waypoints
        if not self.marble.wp_opts[self.marble.wp_state]['folder_name'] == '':
            if self.marble.wp_opts[self.marble.wp_state]['is_rrt'] and len(self.waypoints) == 0:#self.marble.wp_opts[self.marble.wp_state]['needs_render']:
                # create rrt file and reload
                parent = self.marble.wp_opts[self.marble.wp_state]['parent']
                old_folder_name = self.marble.wp_opts[parent]['folder_name']
                old_file_path = os.path.join(PWD, 'resources', 'wp_data', old_folder_name, '%s_%s.txt' % (self._home_map, old_folder_name))
                new_folder_name = self.marble.wp_opts[self.marble.wp_state]['folder_name']
                new_file_path = os.path.join(PWD, 'resources', 'wp_data', new_folder_name, '%s_%s.txt' % (self._home_map, new_folder_name))
                write_waypoints(old_file_path, new_file_path)
                self.load_wp_from_file()
                self.compile_NED_waypoints()
                self.marble.current_path_NE_list = get_full_current_path(self.NED_waypoints)
                #self.marble.wp_opts[self.marble.wp_state]['needs_render'] = False
            else:
                self.compile_NED_waypoints()
                self.marble.current_path_NE_list = get_full_current_path(self.NED_waypoints)
        self.update_lists() # update wp_window contents
        self.set_title()
        for i, wp in enumerate(self.waypoints): # update map waypoints
            self.marble.WPH.emit_inserted(wp[0], wp[1], wp[2], i)

    # UPDATE FUNCTIONS

    def compile_NED_waypoints(self):
        for wp in self.waypoints:
            meter_data = self.marble.GB.gps_to_ned(wp[0],wp[1], (wp[2]-22.0)/3.28084)
            self.NED_waypoints.append([meter_data[0], meter_data[1], meter_data[2], wp[3]])

    def set_title(self): # needs new home map and wp_state
        title_substr = self.marble.wp_opts[self.marble.wp_state]['title_substr']
        title_string = '%s %s' % (self._home_map, title_substr)
        self.waypoint_label.setText(QString(title_string))

    def load_wp_from_file(self): # needs new home map and wp_state
        if not self.marble.wp_opts[self.marble.wp_state]['folder_name'] == '':
            folder_name = self.marble.wp_opts[self.marble.wp_state]['folder_name']
            self.waypoints = map_info_parser.get_typed_waypoints(self._home_map, folder_name)
        else:
            self.waypoints = []

    def update_lists(self): # needs new self.waypoints
        self.listWidget.clear()
        i = 0
        for waypoint in self.waypoints:
            self.listWidget.addItem(QString(str(i)+': '+str(waypoint)))
            i += 1

    def transfer_waypoint_data(self): # needs new self.waypoints
        if self.marble.wp_state == 'DropWP' and len(self.waypoints) > 0:
            # only expecting to have 1 waypoint
            wp = self.waypoints[0]
            lat = wp[0]
            lon = wp[1]
            # altitude not used!
            angle = wp[3]
            wind_n = self.Vw_comps[0]
            print 'wind_n:', wind_n # ------------------------------
            wind_e = self.Vw_comps[1]
            print 'wind_e:', wind_e # ------------------------------
            # the following class will automatically publish the drop waypoints:
            drop_wp_publisher = drop_plan(lat, lon, angle, wind_n, wind_e)
        elif self.marble.GIS.received_msg and not self.marble.wp_state == 'None':
            wp = self.waypoints[0]
            meter_data = self.marble.GIS.GB.gps_to_ned(wp[0],wp[1], (wp[2]-22.0)/3.28084)
            self.WPP.publish_wp_to_plane(meter_data, wp[3], True, False)
            for wp in self.waypoints[1:]:
                meter_data = self.marble.GIS.GB.gps_to_ned(wp[0], wp[1], (wp[2]-22.0)/3.28084)
                self.WPP.publish_wp_to_plane(meter_data, wp[3], False, False)
            #self.WPP.publish_wp_to_plane([0, -50, -60], 0.0, False, False)

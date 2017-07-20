from python_qt_binding import loadUi
from PyQt5.Qt import *
from PyQt5 import QtGui

QString = type("")

import os
import map_info_parser
import rospy
from std_msgs.msg import Bool

# import Michael's landing script
from landing_planner import publishwaypoints

PWD = os.path.dirname(os.path.abspath(__file__))
RTH_ALT = 10 # "return to home" command will have the plane fly 10 m above home pt

class CmWindow(QWidget):

    drop_pub = rospy.Publisher('bomb_drop', Bool, queue_size=5)

    def __init__(self, marble, uifname = 'cm_window.ui'):
        super(CmWindow, self).__init__()
        self.marble = marble
        ui_file = os.path.join(PWD, 'resources', uifname)
        loadUi(ui_file, self)
        self.setObjectName(uifname)

        self.zero_sensor_button.clicked.connect(self.zero_sensor_command)
        self.takeoff_button.clicked.connect(self.takeoff_command)
        self.loiter_button.clicked.connect(self.loiter_command)
        self.rth_button.clicked.connect(self.rth_command)
        self.land_button.clicked.connect(self.land_command)
        self.drop_button.clicked.connect(self.drop_command)
        self.marble.WPH.wp_clicked.connect(self.clicked_waypoint)

    def zero_sensor_command(self):
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        print('zero sensor functionality pending')

    def takeoff_command(self):
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        print ('takeoff functionality pending')

    def loiter_command(self):
        pass
        '''
        if self.marble.GIS.received_msg:
            try:
                lat = float(str(self.loiter_lat_field.toPlainText()))
                lon = float(str(self.loiter_lon_field.toPlainText()))
                alt = float(str(self.loiter_alt_field.toPlainText()))
                meter_data = self.marble.GIS.GB.gps_to_ned(lat, lon, alt/3.281)
                #self.OWPP.publish_wp_to_plane(meter_data) ============================================
            except ValueError:
                print('Incorrectly formatted fields. Must all be numbers.')
        '''

    def rth_command(self):
        pass
        '''
        if self.marble.GIS.received_msg:
            lat = self.marble.latlon[0]
            lon = self.marble.latlon[1]
            meter_data = self.marble.GIS.GB.gps_to_ned(lat, lon, RTH_ALT)
            #self.OWPP.publish_wp_to_plane(meter_data) =================================================
        '''

    def land_command(self): # ++++++++++++++++++++++++++++++++++++++++++++++
        if self.marble.GIS.received_msg:
            try:
                lat = float(str(self.land_lat_field.toPlainText()))
                lon = float(str(self.land_lon_field.toPlainText()))
                chi = float(str(self.land_chi_field.toPlainText()))
                direction = float(str(self.land_direction_field.toPlainText()))
                meter_data = self.marble.GIS.GB.gps_to_ned(lat, lon, 0.0) # necessary ?
                publishwaypoints(meter_data[0], meter_data[1], chi, direction)
            except ValueError:
                print('Incorrectly formatted fields. Must all be numbers.')

    def drop_command(self):
        self.drop_pub.publish(True)

    def clicked_waypoint(self, lat, lon):
        self.land_lat_field.setText(QString(str(lat)))
        self.land_lon_field.setText(QString(str(lon)))

    def closeEvent(self, QCloseEvent):
        #self.marble.setInputEnabled(True)
        self.marble._mouse_attentive = False

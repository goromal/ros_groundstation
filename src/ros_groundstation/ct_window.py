from python_qt_binding import loadUi
from PyQt5.Qt import *
from PyQt5.QtGui import *
QString = type("")

import os, rospy

from .map_subscribers import *

PWD = os.path.dirname(os.path.abspath(__file__))

class CtWindow(QWidget):
    def __init__(self, uifname = 'create_task.ui'):
        super(CtWindow, self).__init__()
        ui_file = os.path.join(PWD, 'resources', uifname)
        loadUi(ui_file, self)
        self.setObjectName(uifname)

        self.mission_dict = {'Waypoint Mission': 0, 'Payload Mission': 1,
                             'Search Mission': 2, 'Off-Axis Detection': 6,
                             'Loiter Mission': 7} # MUST BE ASCENDING ORDER!

        self.comboBox.setCurrentIndex(sorted(self.mission_dict.values()).index(PPSub.mission_type))
        self.comboBox.currentIndexChanged[str].connect(self.change_mission)
        self.bwp_button.clicked.connect(self.get_base_waypoints)
        self.pwp_button.clicked.connect(self.get_path_waypoints)
        self.awp_button.clicked.connect(self.approve_waypoints)

    def change_mission(self):
        mission_name = self.comboBox.currentText()
        PPSub.changeMissionType(self.mission_dict[mission_name])

    def get_base_waypoints(self):
        PPSub.getBaseWaypoints()

    def get_path_waypoints(self):
        if len(PPSub.base_wps) == 0:
            PPSub.getBaseWaypoints()
        PPSub.getPath()

    def approve_waypoints(self):
        if len(PPSub.path_wps) > 0:
            PPSub.approvePath()

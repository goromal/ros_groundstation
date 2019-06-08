from python_qt_binding import loadUi
from PyQt5.Qt import *
from PyQt5.QtGui import *
QString = type("")

import os, rospy

from .map_subscribers import *
# from .Signals import AttentiveHandler

PWD = os.path.dirname(os.path.abspath(__file__))

class CtWindow(QWidget):
    def __init__(self, marble, uifname = 'create_task.ui'):
        super(CtWindow, self).__init__()
        ui_file = os.path.join(PWD, 'resources', uifname)
        loadUi(ui_file, self)
        self.setObjectName(uifname)
        self.marble = marble

        self.mission_dict = {'Waypoint Mission': 0, 'Payload Mission': 1,
                             'Search Mission': 2, 'OTHER': 3, 'Land': 4,
                             'Emergent': 5, 'Off-Axis Detection': 6,
                             'Loiter Mission': 7} # MUST BE ASCENDING ORDER!

        self.comboBox.setCurrentIndex(sorted(self.mission_dict.values()).index(PPSub.mission_type))
        self.comboBox.currentIndexChanged[str].connect(self.change_mission)
        self.pwp_button.clicked.connect(self.get_path_waypoints)
        self.awp_button.clicked.connect(self.approve_waypoints)
        self.pushButton_2.clicked.connect(self.handleLandingEndClicked)
        self.pushButton.clicked.connect(self.handleLandingDirectionClicked)
        self.pushButton_3.clicked.connect(self.handleResetLandingWaypointsClicked)

    def handleLandingEndClicked(self):
        if PPSub.mission_type == self.mission_dict['Land']:
            self.marble.activateAttentive(0)
            # AttentiveHandler.emitAttentiveActivated(0)

    def handleLandingDirectionClicked(self):
        if PPSub.mission_type == self.mission_dict['Land']:
            self.marble.activateAttentive(1)
            # AttentiveHandler.emitAttentiveActivated(1)

    def handleResetLandingWaypointsClicked(self):
        PPSub.resetLandingWaypoints()
        self.marble.deactivateAttentive()
        # AttentiveHandler.emitAttentiveDeactivated()

    def change_mission(self):
        mission_name = self.comboBox.currentText()
        PPSub.changeMissionType(self.mission_dict[mission_name])

    def get_path_waypoints(self):
        PPSub.getPath()

    def approve_waypoints(self):
        if len(PPSub.path_wps) > 0:
            PPSub.approvePath()

    def closeEvent(self, QCloseEvent):
        self.marble.deactivateAttentive()

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from .marble_map import MarbleMap
from .ct_window import CtWindow
from .op_window import OpWindow
import map_info_parser
import os

from PyQt5.QtGui import *
from PyQt5.QtCore import *

PWD = os.path.dirname(os.path.abspath(__file__))

class MapWindow(QWidget):
    def __init__(self, uifname = 'map_widget_TEMPDEVEL.ui'):
        super(MapWindow, self).__init__()
        button_icon_file = os.path.join(PWD, 'resources', 'airplane.png')
        ui_file = os.path.join(PWD, 'resources', uifname)
        loadUi(ui_file, self)
        self.setObjectName(uifname)

        self.gps_dict = map_info_parser.get_gps_dict()
        self.blankname = '-- BLANK MAP --'
        self.gps_dict[self.blankname] = [[0.0, 0.0], 18]
        self._marble_map = MarbleMap(self.gps_dict, self.blankname)
        self.verticalLayout.addWidget(self._marble_map)

        self._home_opts.clear()
        keylist = sorted(self.gps_dict, key=self.gps_dict.get)
        self._home_opts.addItems(keylist)
        self._home_opts.setCurrentIndex(keylist.index(map_info_parser.get_default()))
        self._home_opts.currentIndexChanged[str].connect(self._update_home)

        self._gridview_toggle.stateChanged[int].connect(self._marble_map.grid_viewer_toggle)

        self.init_ct_window()
        self.init_op_window()
        self._recenter.clicked.connect(self._marble_map.recenter)
        self._get_mission.clicked.connect(self._marble_map.get_mission)

    def init_ct_window(self):
        self.ctWindow = CtWindow(self._marble_map)
        self._create_task.clicked.connect(self.open_ct_window)

    def init_op_window(self):
        self.opWindow = OpWindow(self._marble_map)
        self._map_options.clicked.connect(self.open_op_window)

    def open_ct_window(self):
        self.ctWindow.show()

    def open_op_window(self):
        self.opWindow.show()

    def _update_home(self):
        self._marble_map.change_home(self._home_opts.currentText())

    def closeEvent(self, event):
        self.opWindow.close()
        self.wpWindow.close()
        self.cmWindow.close()
        super(MapWindow, self).close()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

#from .manage_kml import ManageKML
from .marble_map import MarbleMap
from .op_window import OpWindow
from .wp_window import WpWindow
from .cm_window import CmWindow
import map_info_parser
import os

from PyQt5.QtGui import *
from PyQt5.QtCore import *

PWD = os.path.dirname(os.path.abspath(__file__))

class MapWindow(QWidget):
    def __init__(self, uifname = 'map_widget.ui'):
        super(MapWindow, self).__init__()
        button_icon_file = os.path.join(PWD, 'resources', 'airplane.png')
        ui_file = os.path.join(PWD, 'resources', uifname)
        loadUi(ui_file, self)
        self.setObjectName(uifname)

        self._marble_map = MarbleMap()
        self.verticalLayout.addWidget(self._marble_map)

        map_coords = map_info_parser.get_gps_dict()
        self._home_opts.clear()
        self._home_opts.addItems(list(map_coords))
        self._home_opts.setCurrentIndex(list(map_coords).index(map_info_parser.get_default()))
        self._home_opts.currentIndexChanged[str].connect(self._update_home)

        self._pathviewer_toggle.stateChanged[int].connect(self._marble_map.path_viewer_toggle)

        #self.init_manage_kml()
        self.init_op_window()
        self.init_wp_window()
        self.init_cm_window()

    def init_manage_kml(self):
        self.manageKML = ManageKML(self._marble_map)
        self._manage_KML.clicked.connect(self.manageKML.display_manage_KML_modal)
        self.manageKML.add_default_KML_files()

    def init_op_window(self):
        self.opWindow = OpWindow(self._marble_map)
        self._map_options.clicked.connect(self.open_op_window)

    def init_wp_window(self):
        self.wpWindow = WpWindow(self._marble_map)
        self._send_WP.clicked.connect(self.open_wp_window)

    def init_cm_window(self):
        self.cmWindow = CmWindow(self._marble_map)
        self._special_commands.clicked.connect(self.open_cm_window)

    def open_op_window(self):
        self.opWindow.show()

    def open_wp_window(self):
        self.wpWindow.show()

    def open_cm_window(self):
        self._marble_map._mouse_attentive = True
        self.cmWindow.show()

    def _update_home(self):
        self._marble_map.change_home(self._home_opts.currentText())

    def closeEvent(self, event): # ++++++++++++++
        self.opWindow.close()
        self.wpWindow.close()
        self.cmWindow.close()
        super(MapWindow, self).close()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

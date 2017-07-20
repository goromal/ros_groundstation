from python_qt_binding import loadUi
from PyQt5 import QtGui
import os

PWD = os.path.dirname(os.path.abspath(__file__))

class ManageKML(QtGui.QDialog):
    def __init__(self, _marble_map, uifname = 'manage_kml_dialog.ui'):
        super(ManageKML, self).__init__()
        self._marble_map = _marble_map
        ui_file = os.path.join(PWD, 'resources', uifname)
        loadUi(ui_file, self)
        self.setObjectName(uifname)
        self.setup_ui()

    def setup_ui(self):
        self.addButton.clicked.connect(self.display_add_KML_dialog)

    def display_manage_KML_modal(self):
        self.exec_()

    def add_default_KML_files(self):
        defaultDirectory = os.path.join(PWD, 'resources', 'kml_files')
        for file in os.listdir(defaultDirectory):
            if (file[-4:] == '.kml'):
                file_path = os.path.join(defaultDirectory, file)
                self.add_kml_file(file_path)

    def display_add_KML_dialog(self):
        result = QtGui.QFileDialog.getOpenFileName(self, 'Open file', PWD, '*.kml')
        fname = result[0]
        if (fname[-4:] == '.kml'):
            self.add_kml_file(fname)

    def add_kml_file(self, file_path):
         self._marble_map.model().addGeoDataFile(file_path)

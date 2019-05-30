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

# WHAT TO DO WHEN A PATH IS APPROVED???? ++++

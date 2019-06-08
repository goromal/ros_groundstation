from PyQt5.QtCore import QObject, pyqtSignal

# class AttentiveHandler(QObject):
#     QObject.__init__()
#     attentiveActivated = pyqtSignal(int)
#     attentiveDeactivated = pyqtSignal()
#     waypointClicked = pyqtSignal(float, float)
#     @staticmethod
#     def emitAttentiveActivated(idx):
#         AttentiveHandler.attentiveActivated.emit(idx)
#     @staticmethod
#     def emitAttentiveDeactivated():
#         AttentiveHandler.attentiveDeactivated.emit()
#     @staticmethod
#     def emitWaypointClicked(lat, lon):
#         AttentiveHandler.waypointClicked.emit(lat, lon)

class WP_Handler(QObject):
    # # Signal for notifying that attentive mode entered
    # at_activated = pyqtSignal()
    #
    # # Signal for notifying that attentive mode left
    # at_deactivated = pyqtSignal()
    #
    # # Signal for clicked waypoint with args (float lat, float lon)
    # wp_clicked = pyqtSignal(float, float)

    # Signal for inserted waypoint with args (float lat, float lon, float alt, int position)
    # Position is inserted position in flight path list
    wp_inserted = pyqtSignal(float, float, float, int)

    # Signal for removing waypoint with arg (int position)
    wp_removed = pyqtSignal(int)

    # Signal for notifying a change in map home, with arg (string new_home)
    home_changed = pyqtSignal(str)

    # def emit_clicked(self, lat, lon):
    #     self.wp_clicked.emit(lat, lon)

    def emit_inserted(self, lat, lon, alt, pos):
        self.wp_inserted.emit(lat, lon, alt, pos)

    def emit_removed(self, pos):
        self.wp_removed.emit(pos)

    def emit_home_change(self, new_home):
        self.home_changed.emit(new_home)

from .ground_station import GroundStationWidget
from qt_gui.plugin import Plugin


class GroundStationPlugin(Plugin):

    def __init__(self, context):
        super(GroundStationPlugin, self).__init__(context)

        if context.serial_number() > 1:
            raise RuntimeError("Due to a limitation of tf_frames you may not run more than one instance of ground_station.")
        self._widget = GroundStationWidget()
        
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        self.setObjectName('ros_groundstation')

    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)

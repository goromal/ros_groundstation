import os
import rospkg
import roslib

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QAction, QMenu, QWidget, QMessageBox

import rospy

from rqt_py_common import topic_helpers

from .rosplot import ROSData, RosPlotException

from .map_subscribers import *
PWD = os.path.dirname(os.path.abspath(__file__))

def get_plot_fields(topic_name):
    topic_type, real_topic, _ = topic_helpers.get_topic_type(topic_name)
    if topic_type is None:
        message = "topic %s does not exist" % ( topic_name )
        return [], message
    field_name = topic_name[len(real_topic)+1:]

    slot_type, is_array, array_size = roslib.msgs.parse_type(topic_type)
    field_class = roslib.message.get_message_class(slot_type)

    fields = [f for f in field_name.split('/') if f]

    for field in fields:
        # parse the field name for an array index
        try:
            field, _, field_index = roslib.msgs.parse_type(field)
        except roslib.msgs.MsgSpecException:
            message = "invalid field %s in topic %s" % ( field, real_topic )
            return [], message

        if field not in getattr(field_class, '__slots__', []):
            message = "no field %s in topic %s" % ( field_name, real_topic )
            return [], message
        slot_type = field_class._slot_types[field_class.__slots__.index(field)]
        slot_type, slot_is_array, array_size = roslib.msgs.parse_type(slot_type)
        is_array = slot_is_array and field_index is None

        field_class = topic_helpers.get_type_class(slot_type)

    if field_class in (int, float):
        if is_array:
            if array_size is not None:
                message = "topic %s is fixed-size numeric array" % ( topic_name )
                return [ "%s[%d]" % (topic_name, i) for i in range(array_size) ], message
            else:
                message = "topic %s is variable-size numeric array" % ( topic_name )
                return [], message
        else:
            message = "topic %s is numeric" % ( topic_name )
            return [ topic_name ], message
    else:
        if not roslib.msgs.is_valid_constant_type(slot_type):
            numeric_fields = []
            for i, slot in enumerate(field_class.__slots__):
                slot_type = field_class._slot_types[i]
                slot_type, is_array, array_size = roslib.msgs.parse_type(slot_type)
                slot_class = topic_helpers.get_type_class(slot_type)
                if slot_class in (int, float) and not is_array:
                    numeric_fields.append(slot)
            message = ""
            if len(numeric_fields) > 0:
                message = "%d plottable fields in %s" % ( len(numeric_fields), topic_name )
            else:
                message = "No plottable fields in %s" % ( topic_name )
            return [ "%s/%s" % (topic_name, f) for f in numeric_fields ], message
        else:
            message = "Topic %s is not numeric" % ( topic_name )
            return [], message

def is_plottable(topic_names): # returns plottable, message
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    fields, message = get_plot_fields(topic_names)
    return len(fields) > 0, message

def get_topic(topic_tuple):
    key = topic_tuple[0]
    maintopic = ''
    subtopic = topic_tuple[1]
    #if key == 'i':
    #    maintopic = InitSub.getGPSInitTopic()
    if key == 's':
        maintopic = StateSub.getStateTopic()
    #if key == 'r':
    #    maintopic = RCSub.
    if key == 'ci':
        maintopic = ConInSub.getConInTopic()
    if key == 'cc':
        maintopic = ConComSub.getConComTopic()

    if maintopic is None:
        return None
    else:
        return '%s/%s' % (maintopic, subtopic)

class PlotWidget(QWidget):
    _redraw_interval = 40

    def __init__(self, initial_topics=None, start_paused=False):
        super(PlotWidget, self).__init__()
        self.setObjectName('PlotWidget')

        # Available ros topics for plotting
        # keys are i=InitSub, s=StateSub, r=RCSub, p=PathSub, w=WaypointSub,
        #          o=ObstacleSub, g=GPSDataSub, ci=ConInSub, cc=ConComSub
        self.message_dict = {
            'Course angle vs. Commanded':[('cc','chi_c'),('s','chi')],
            'Roll angle vs. Commanded':[('ci','phi_c'),('s','phi')],
            'Pitch angle vs. Commanded':[('s','theta'),('ci','theta_c')],
            'Airspeed vs. Commanded':[('s','Va'),('cc','Va_c')]
            }

        # # Available ros topics for plotting
        # self.message_dict = {
        #     'Course angle (rad)':'/state/chi',
        #     'Course angle commanded (rad)':'/controller_commands/chi_c',
        #     'Airspeed (m/s)':'/state/Va',
        #     'Angle of attack (rad)':'/state/alpha',
        #     'Slide slip angle (rad)':'/state/beta',
        #     'Roll angle (rad)':'/state/phi',
        #     'Pitch angle (rad)':'/state/theta',
        #     'Yaw angle (rad)':'/state/psi',
        #     'Body frame rollrate (rad/s)':'/state/p',
        #     'Body frame pitchrate (rad/s)':'/state/q',
        #     'Body frame yawrate (rad/s)':'/state/r',
        #     'Groundspeed (m/s)':'/state/Vg'
        #     }

        #self._initial_topics = initial_topics

        rp = rospkg.RosPack()
        ui_file = os.path.join(PWD, 'resources', 'plot.ui')
        loadUi(ui_file, self)

        self.pause_button.setIcon(QIcon.fromTheme('media-playback-pause'))
        self.clear_button.setIcon(QIcon.fromTheme('edit-clear'))
        self.data_plot = None

        if start_paused:
            self.pause_button.setChecked(True)

        self._start_time = rospy.get_time()
        self._rosdata = {}

        self._current_key = ''
        self._current_topics = []

        self._remove_topic_menu = QMenu()

        self._msgs.clear()
        self._msgs.addItems(self.message_dict.keys())

        self._msgs.currentIndexChanged[str].connect(self._draw_graph) # <<<<<<< start here (also modify the dict)

        # init and start update timer for plot
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.update_plot)

    def switch_data_plot_widget(self, data_plot):
        self.enable_timer(enabled=False)

        self.data_plot_layout.removeWidget(self.data_plot)
        if self.data_plot is not None:
            self.data_plot.close()

        self.data_plot = data_plot
        self.data_plot_layout.addWidget(self.data_plot)
        self.data_plot.autoscroll(True)

        # setup drag 'n drop
        self.data_plot.dropEvent = self.dropEvent
        self.data_plot.dragEnterEvent = self.dragEnterEvent
        '''
        if self._initial_topics:
            for topic_name in self._initial_topics:
                self.add_topic(topic_name)
            self._initial_topics = None
        else:
            for topic_name, rosdata in self._rosdata.items():
                data_x, data_y = rosdata.next()
                self.data_plot.add_curve(topic_name, topic_name, data_x, data_y)

        self._subscribed_topics_changed()
        '''

    def _draw_graph(self):
        #print 'entering _draw_graph'
        # NOT GONNA WORRY ABOUT THIS!
        # plottable, message = is_plottable(self.message_dict[self._msgs.currentText()]) # <<<<<< will feed this a list

        if self._current_key and not (self._msgs.currentText() == self._current_key):
            for topic_tuple in self._current_topics:
                #print 'removing %s from plot' % topic # ------------------------
                self.remove_topic(topic_tuple[0], topic_tuple[1])
            self._current_topics = []

        self._current_topics = []

        self._current_key = self._msgs.currentText()
        #print 'current key:', self._current_key
        for topic_tuple in self.message_dict[self._current_key]:
            #topic = get_topic(topic_tuple)
            #print 'topic:', topic
            #if not (topic is None):
            self._current_topics.append(topic_tuple)
            #print 'adding %s to plot' % topic # --------------------------
            self.add_topic(topic_tuple[0], topic_tuple[1])
        self._subscribed_topics_changed()

    @Slot(bool)
    def on_pause_button_clicked(self, checked):
        self.enable_timer(not checked)

    @Slot()
    def on_clear_button_clicked(self):
        self.clear_plot()

    def update_plot(self):
        if self.data_plot is not None:
            needs_redraw = False
            for topic_name, rosdata in self._rosdata.items():
                try:
                    data_x, data_y = rosdata.next()
                    if data_x or data_y:
                        self.data_plot.update_values(topic_name, data_x, data_y)
                        needs_redraw = True
                except RosPlotException as e:
                    qWarning('PlotWidget.update_plot(): error in rosplot: %s' % e)
            if needs_redraw:
                self.data_plot.redraw()

    def _subscribed_topics_changed(self):
        #self._update_remove_topic_menu() # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        if not self.pause_button.isChecked():
            # if pause button is not pressed, enable timer based on subscribed topics
            self.enable_timer(self._rosdata)
        self.data_plot.redraw()

    def _update_remove_topic_menu(self):
        def make_remove_topic_function(x):
            return lambda: self.remove_topic(x)

        self._remove_topic_menu.clear()
        for topic_name in sorted(self._rosdata.keys()):
            action = QAction(topic_name, self._remove_topic_menu)
            action.triggered.connect(make_remove_topic_function(topic_name))
            self._remove_topic_menu.addAction(action)

        if len(self._rosdata) > 1:
            all_action = QAction('All', self._remove_topic_menu)
            all_action.triggered.connect(self.clean_up_subscribers)
            self._remove_topic_menu.addAction(all_action)

    def add_topic(self, topic_code, topic_item):
        topics_changed = False
        topic_name = topic_code + '/' + topic_item
        #for topic_name in get_plot_fields(topic_name)[0]: # <<<<<<<< this is what allows for multiple topics
        if topic_name in self._rosdata:
            qWarning('PlotWidget.add_topic(): topic already subscribed: %s' % topic_name)
            #continue
            return

        self._rosdata[topic_name] = ROSData(topic_code, topic_item, self._start_time)

        if self._rosdata[topic_name].error is not None:
            qWarning(str(self._rosdata[topic_name].error))
            del self._rosdata[topic_name]
        else:
            data_x, data_y = self._rosdata[topic_name].next()
            self.data_plot.add_curve(topic_name, topic_name, data_x, data_y)
            #print self._rosdata.items() # ------------------------------------------
            topics_changed = True

        #if topics_changed:
        #    self._subscribed_topics_changed()

    def remove_topic(self, topic_code, topic_item):
        topic_name = topic_code + '/' + topic_item
        self._rosdata[topic_name].close()
        del self._rosdata[topic_name]
        self.data_plot.remove_curve(topic_name)

        self._subscribed_topics_changed()

    def clear_plot(self):
        for topic_name, _ in self._rosdata.items():
            self.data_plot.clear_values(topic_name)
        self.data_plot.redraw()

    def clean_up_subscribers(self):
        for topic_name, rosdata in self._rosdata.items():
            rosdata.close()
            self.data_plot.remove_curve(topic_name)
        self._rosdata = {}

        self._subscribed_topics_changed()

    def enable_timer(self, enabled=True):
        if enabled:
            self._update_plot_timer.start(self._redraw_interval)
        else:
            self._update_plot_timer.stop()

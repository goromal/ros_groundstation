import string
import sys, os
import threading
import time

import rosgraph
import roslib.message
import roslib.names
import rospy

from .map_subscribers import *
from python_qt_binding.QtCore import QTimer

class RosPlotException(Exception):
    pass


def _get_topic_type(topic):
    """
    subroutine for getting the topic type
    (nearly identical to rostopic._get_topic_type, except it returns rest of name instead of fn)
    :returns: topic type, real topic name, and rest of name referenced
      if the topic points to a field within a topic, e.g. /rosout/msg, ``str, str, str``
    """
    try:
        master = rosgraph.Master('/rosplot')
        val = master.getTopicTypes()
    except:
        raise RosPlotException("unable to get list of topics from master")
    matches = [(t, t_type) for t, t_type in val if t == topic or topic.startswith(t + '/')]
    if matches:
        t, t_type = matches[0]
        if t_type == roslib.names.ANYTYPE:
            return None, None, None
        if t_type == topic:
            return t_type, None
        return t_type, t, topic[len(t):]
    else:
        return None, None, None


def get_topic_type(topic):
    """
    Get the topic type (nearly identical to rostopic.get_topic_type, except it doesn't return a fn)
    :returns: topic type, real topic name, and rest of name referenced
      if the \a topic points to a field within a topic, e.g. /rosout/msg, ``str, str, str``
    """
    topic_type, real_topic, rest = _get_topic_type(topic)
    if topic_type:
        return topic_type, real_topic, rest
    else:
        return None, None, None

'''
class ROSData(object):
    """
    Subscriber to ROS topic that buffers incoming data
    """

    def __init__(self, topic, start_time):
        self.name = topic
        self.start_time = start_time
        self.error = None

        self.lock = threading.Lock()
        self.buff_x = []
        self.buff_y = []

        topic_type, real_topic, fields = get_topic_type(topic)
        if topic_type is not None: #============================================
            self.field_evals = generate_field_evals(fields)
            data_class = roslib.message.get_message_class(topic_type)
            self.sub = rospy.Subscriber(real_topic, data_class, self._ros_cb)
            #self.sub = rospy.Subscriber(topic, data_class, self._ros_cb)
            #print 'plot subscribed to', topic
        #=======================================================================
        else:
            self.error = RosPlotException("Can not resolve topic type of %s" % topic)

    def close(self):
        self.sub.unregister()

    def _ros_cb(self, msg):
        print 'callback entered for', self.name # --------------------------------------------------------------------------------
        """
        ROS subscriber callback
        :param msg: ROS message data
        """
        try:
            self.lock.acquire()
            try:
                self.buff_y.append(self._get_data(msg))
                # #944: use message header time if present
                if msg.__class__._has_header:
                    self.buff_x.append(msg.header.stamp.to_sec() - self.start_time)
                else:
                    self.buff_x.append(rospy.get_time() - self.start_time)
                #self.axes[index].plot(datax, buff_y)
                #print 'added', self.name, 'data, x,y=', str(rospy.get_time() - self.start_time), str(self._get_data(msg)) # -----------------------------
            except AttributeError, e:
                #print 'invalid' # ---------------------------------------------------------------------------------------------------------------
                self.error = RosPlotException("Invalid topic spec [%s]: %s" % (self.name, str(e)))
        finally:
            self.lock.release()

    def next(self):
        """
        Get the next data in the series
        :returns: [xdata], [ydata]
        """
        if self.error:
            raise self.error
        try:
            self.lock.acquire()
            buff_x = self.buff_x
            buff_y = self.buff_y
            self.buff_x = []
            self.buff_y = []
        finally:
            self.lock.release()
        return buff_x, buff_y

    def _get_data(self, msg):
        val = msg
        try:
            if not self.field_evals:
                return float(val)
            for f in self.field_evals:
                val = f(val)
            return float(val)
        except IndexError:
            self.error = RosPlotException("[%s] index error for: %s" % (self.name, str(val).replace('\n', ', ')))
        except TypeError:
            self.error = RosPlotException("[%s] value was not numeric: %s" % (self.name, val))
'''
def get_data(topic_code, topic_item):
    if topic_code == 's':
        if topic_item == 'chi':
            return StateSub.chi
        elif topic_item == 'phi':
            return StateSub.phi
        elif topic_item == 'theta':
            return StateSub.theta
        elif topic_item == 'Va':
            return StateSub.Va
    elif topic_code == 'ci':
        if topic_item == 'phi_c':
            return ConInSub.phi_c
        elif topic_item == 'theta_c':
            return ConInSub.theta_c
    elif topic_code == 'cc':
        if topic_item == 'chi_c':
            return ConComSub.chi_c
        elif topic_item == 'Va_c':
            return ConComSub.Va_c

class ROSData(object):
    """
    Subscriber to ROS topic that buffers incoming data
    """

    def __init__(self, topic_code, topic_item, start_time):
        self.name = topic_code + '/' + topic_item
        self.start_time = start_time
        self.error = None

        self.lock = threading.Lock()
        self.buff_x = []
        self.buff_y = []

        self.interval = 100     # in milliseconds, period of regular update
        self.timer = QTimer()
        self.timer.setInterval(self.interval)
        self.timer.timeout.connect(self._ros_cb)

        self.code = topic_code
        self.item = topic_item
        #self.data = 0.0
        # go through options and decide what your self.data will be, given the ros subscribers


        self.timer.start()

    def close(self):
        self.timer.stop()

    def _ros_cb(self):
        #print 'callback entered for', self.name # --------------------------------------------------------------------------------
        """
        ROS "subscriber callback"
        :param msg: ROS message data
        """
        self.buff_x.append(rospy.get_time() - self.start_time)
        self.buff_y.append(get_data(self.code, self.item))

    def next(self):
        """
        Get the next data in the series
        :returns: [xdata], [ydata]
        """
        if self.error:
            raise self.error
        try:
            self.lock.acquire()
            buff_x = self.buff_x
            buff_y = self.buff_y
            self.buff_x = []
            self.buff_y = []
        finally:
            self.lock.release()
        return buff_x, buff_y

def _array_eval(field_name, slot_num):
    """
    :param field_name: name of field to index into, ``str``
    :param slot_num: index of slot to return, ``str``
    :returns: fn(msg_field)->msg_field[slot_num]
    """
    def fn(f):
        return getattr(f, field_name).__getitem__(slot_num)
    return fn


def _field_eval(field_name):
    """
    :param field_name: name of field to return, ``str``
    :returns: fn(msg_field)->msg_field.field_name
    """
    def fn(f):
        return getattr(f, field_name)
    return fn


def generate_field_evals(fields):
    try:
        evals = []
        fields = [f for f in fields.split('/') if f]
        for f in fields:
            if '[' in f:
                field_name, rest = f.split('[')
                slot_num = string.atoi(rest[:rest.find(']')])
                evals.append(_array_eval(field_name, slot_num))
            else:
                evals.append(_field_eval(f))
        return evals
    except Exception, e:
        raise RosPlotException("cannot parse field reference [%s]: %s" % (fields, str(e)))

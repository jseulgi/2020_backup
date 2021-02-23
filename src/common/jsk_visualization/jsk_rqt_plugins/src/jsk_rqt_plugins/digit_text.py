from rqt_gui_py.plugin import Plugin

import python_qt_binding.QtGui as QtGui
import python_qt_binding.QtWidgets as QtWidgets

from QtWidgets import QAction, QMenu, QWidget, QMessageBox, QSizePolicy, QLabel, QComboBox, QLCDNumber
from QtGui import QIcon, QPainter, QColor, QFont, QBrush, QPen

from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot, QEvent, QSize
from threading import Lock
import rospy
import python_qt_binding.QtCore as QtCore
from std_msgs.msg import Bool, Time
import math
from resource_retriever import get_filename
import yaml
import os, sys

from std_msgs.msg import String
from image_view2_wrapper import ComboBoxDialog

class DigitText(Plugin):
    def __init__(self, context):
        super(DigitText, self).__init__(context)
        self.setObjectName("DigitText")
        self._widget = DigitTextWidget()
        context.add_widget(self._widget)
    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)
    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)
    def trigger_configuration(self):
        self._widget.trigger_configuration()

class DigitTextWidget(QWidget):
    _FONT_COLOR = QColor(80,80,80,255)

    def __init__(self):
        super(DigitTextWidget, self).__init__()
        self.lock = Lock()
        self._str_topics = []
        self._str_data = ""
        self._str_sub = None
        self._update_topic_timer = QTimer(self)
        self._update_topic_timer.timeout.connect(self.updateTopics)
        self._update_topic_timer.start(1000)
        self._active_topic = None
        self._dialog = ComboBoxDialog()
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.redraw)
        self._update_plot_timer.start(1000 / 15)

    def redraw(self):
        self.update()

    def paintEvent(self, event):
        with self.lock:
            qp = QPainter()
            qp.begin(self)

            if self._active_topic == "/Status_string":
                qp.setFont(QFont('Helvetica', 20))
                qp.setPen(QPen(QBrush(self._FONT_COLOR), 10))
                qp.drawText(20, 30, self._str_data)

            elif self._active_topic == "/Vehicle_Speed":
                qp.setFont(QFont('Helvetica', 20))
                qp.setPen(QPen(QBrush(self._FONT_COLOR), 10))

                qp.drawText(20, 30, "Speed : " + self._str_data.rjust(3, '0') + " km/h")       

            elif self._active_topic == "/Current_Steer_Angle":
                qp.setFont(QFont('Helvetica', 20))
                qp.setPen(QPen(QBrush(self._FONT_COLOR), 10))

                if self._str_data != "" and self._str_data[0] == '-':
                    qp.drawText(20, 30, "Steer Angle : " + self._str_data + " degree")
                else:
                    qp.drawText(20, 30, "Steer Angle : " + '+' + self._str_data + " degree")

            qp.end()

            return

    def trigger_configuration(self):
        self._dialog.exec_()
        self.setupSubscriber(self._str_topics[self._dialog.number])

    def updateTopics(self):
        need_to_update = False
        for (topic, topic_type) in rospy.get_published_topics():
            if topic_type == "std_msgs/String":
                if not topic in self._str_topics:
                    self._str_topics.append(topic)
                    need_to_update = True
        if need_to_update:
            self._str_topics = sorted(self._str_topics)
            self._dialog.combo_box.clear()
            for topic in self._str_topics:
                self._dialog.combo_box.addItem(topic)
            if self._active_topic:
                if self._active_topic not in self._str_topics:
                    self._str_topics.append(self._active_topic)
                    self._dialog.combo_box.addItem(self._active_topic)
                self._dialog.combo_box.setCurrentIndex(self._str_topics.index(self._active_topic))

    def setupSubscriber(self, topic):
        if self._str_sub:
            self._str_sub.unregister()
        self._str_sub = rospy.Subscriber(topic, String,
                                           self.strCallback)
        self._active_topic = topic

    def onActivated(self, number):
        self.setupSubscriber(self._str_topics[number])

    def strCallback(self, msg):
        self._str_data = msg.data

    def save_settings(self, plugin_settings, instance_settings):
        if self._active_topic:
            instance_settings.set_value("active_topic", self._active_topic)

    def restore_settings(self, plugin_settings, instance_settings):
        if instance_settings.value("active_topic"):
            topic = instance_settings.value("active_topic")
            self._dialog.combo_box.addItem(topic)
            self.setupSubscriber(topic)

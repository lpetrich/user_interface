#!/usr/bin/env python

from .user_interface import UserInterfaceWidget
from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import Qt, QTimer
import rospy

class UserInterfacePlugin(Plugin):
    def __init__(self, context):
        super(UserInterfacePlugin, self).__init__(context)
        if context.serial_number() > 1:
            raise RuntimeError("You may not run more than one instance of user_interface.")
        self.setObjectName('Robotic Assistance Interface')
        self._widget = UserInterfaceWidget()
        context.add_widget(self._widget)

        self._update_parameter_timer = QTimer(self)
        self._update_parameter_timer.timeout.connect(self._widget.pub_check)
        self._update_parameter_timer.start(100)

    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)
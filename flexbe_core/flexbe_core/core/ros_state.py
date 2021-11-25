#!/usr/bin/env python
from rclpy.exceptions import ParameterNotDeclaredException
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached

from flexbe_core.core.state import State
from flexbe_core.logger import Logger
from flexbe_core.state_logger import StateLogger


class RosState(State):
    """
    A state to interface with ROS.
    """
    _node = None
    _breakpoints = []

    @staticmethod
    def initialize_ros(node):
        RosState._node = node
        try:
            RosState._breakpoints = node.get_parameter('breakpoints')
        except ParameterNotDeclaredException as e:
            node.declare_parameter('breakpoints', [])
            RosState._breakpoints = node.get_parameter('breakpoints')
        # RosState._breakpoints = node.declare_parameter('breakpoints', [])
        ProxyPublisher._initialize(RosState._node)
        ProxySubscriberCached._initialize(RosState._node)
        StateLogger.initialize_ros(RosState._node)
        Logger.initialize(RosState._node)

    def __init__(self, *args, **kwargs):
        super(RosState, self).__init__(*args, **kwargs)
        self._rate = RosState._node.create_rate(50)
        self._is_controlled = False

        self._pub = ProxyPublisher()
        self._sub = ProxySubscriberCached()

    def sleep(self):
        self._rate.sleep()

    @property
    def sleep_duration(self):
        # Check for none to get the start time
        if self._start_time is None:
            self._start_time = RosState._node.get_clock().now()

        elapsed = RosState._node.get_clock().now() - self._start_time

        if elapsed.nanoseconds > self._rate._timer.timer_period_ns:
            # Set back to none for next time
            self._start_time = None
            return 0.0
        else:
            # Sleep duration is now the timer's period minus elapsed time
            return (self._rate._timer.timer_period_ns - elapsed.nanoseconds) * 1e-9
            
        # return self._rate._timer.time_until_next_call() * 1e-9

    def set_rate(self, rate):
        """
        Set the execution rate of this state,
        i.e., the rate with which the execute method is being called.

        Note: The rate is best-effort, real-time support is not yet available.

        @type label: float
        @param label: The desired rate in Hz.
        """
        self._rate.destroy()
        self._rate = RosState._node.create_rate(rate)

    def _enable_ros_control(self):
        self._is_controlled = True

    def _disable_ros_control(self):
        self._is_controlled = False

    @property
    def is_breakpoint(self):
        return self.path in RosState._breakpoints.get_parameter_value().string_array_value

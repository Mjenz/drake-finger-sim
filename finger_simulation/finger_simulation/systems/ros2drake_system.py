"""Leafsystem class connecting ROS topic and drake simulation."""

from finger_interfaces.msg import MotorFeedback

import numpy as np

from pydrake.systems.framework import LeafSystem

import rclpy


class Ros2Drake(LeafSystem):
    """Leaf system connecting ros inputs to drake."""

    def __init__(self, node):
        """Create instance of MotorSystem."""
        super().__init__()
        self.nu = 3

        # save node
        self._node = node

        # Internal state: holds the last received position setpoint (ZOH)
        self.setpoint_state_index = self.DeclareDiscreteState(self.nu)
        self.DeclarePerStepDiscreteUpdateEvent(
            self._update_setpoint)

        # Output port: motor position setpoints (fed into PID desired_state)
        self.DeclareVectorOutputPort(
            'motor_pos_setpoint', self.nu, self._calc_setpoint
        )

        # Set subscriber
        self._latest_setpoint = np.zeros(self.nu)
        self._sub = self._node.create_subscription(
            MotorFeedback,
            '/motor_pos_actual_feedback',
            self._ros_setpoint_callback,
            10,
        )

    def _calc_setpoint(self, context, output):
        """Output motor position setpoints."""
        state = context.get_discrete_state(
            self.setpoint_state_index).get_value()
        output.SetFromVector(state)

    def _ros_setpoint_callback(self, msg):
        """Save new position setpoint messages."""
        data = list(msg.motor_positions)
        if len(data) == 3:
            self._latest_setpoint = np.array(
                (data + [0.0] * self.nu)[:self.nu])

    def _update_setpoint(self, context, discrete_state):
        """Spin ROS node every time there is a simulation update for msgs."""
        rclpy.spin_once(self._node, timeout_sec=0)
        discrete_state.set_value(self._latest_setpoint)

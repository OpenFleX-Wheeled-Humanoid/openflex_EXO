#!/usr/bin/env python3
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class OpenFlexArmBridgeNode(Node):
    LEFT_JOINT_NAMES = [f'openarmx_left_joint{i}' for i in range(1, 8)]
    RIGHT_JOINT_NAMES = [f'openarmx_right_joint{i}' for i in range(1, 8)]

    def __init__(self):
        super().__init__('openflex_arm_bridge_node')
        self.declare_parameter('max_joint_diff_rad', 0.873)
        self.declare_parameter('interpolation_duration', 3.0)
        self.declare_parameter('interpolation_rate_hz', 50.0)
        self.declare_parameter('enable_safety_check', True)
        self.declare_parameter('gripper_scaling_factor', 0.02)

        self.max_joint_diff = float(self.get_parameter('max_joint_diff_rad').value)
        self.interpolation_duration = float(self.get_parameter('interpolation_duration').value)
        self.interpolation_rate = float(self.get_parameter('interpolation_rate_hz').value)
        self.enable_safety_check = bool(self.get_parameter('enable_safety_check').value)
        self.gripper_scale = float(self.get_parameter('gripper_scaling_factor').value)

        self.left_current = None
        self.right_current = None
        self.left_target = None
        self.right_target = None
        self.left_interp = False
        self.right_interp = False
        self.left_interp_start = None
        self.right_interp_start = None
        self.left_interp_target = None
        self.right_interp_target = None
        self.left_interp_time = None
        self.right_interp_time = None
        self.left_ready = False
        self.right_ready = False
        self.left_first_command = True
        self.right_first_command = True
        self.left_no_state_warned = False
        self.right_no_state_warned = False

        self.left_sub = self.create_subscription(JointState, '/left_arm/joint_command', self._left_cb, 10)
        self.right_sub = self.create_subscription(JointState, '/right_arm/joint_command', self._right_cb, 10)
        self.joint_states_sub = self.create_subscription(JointState, '/joint_states', self._joint_states_cb, 10)

        self.left_pub = self.create_publisher(Float64MultiArray, '/left_forward_position_controller/commands', 10)
        self.right_pub = self.create_publisher(Float64MultiArray, '/right_forward_position_controller/commands', 10)

        self.timer = self.create_timer(1.0 / self.interpolation_rate, self._tick)
        self.get_logger().info(
            f'OpenFlex arm bridge started, safety={self.enable_safety_check}, '
            f'max_diff={self.max_joint_diff:.3f}, duration={self.interpolation_duration}s, '
            f'gripper_scale={self.gripper_scale:.4f}'
        )

    def _joint_states_cb(self, msg: JointState):
        left = []
        right = []
        for i in range(1, 8):
            name = f'openarmx_left_joint{i}'
            if name in msg.name:
                left.append(msg.position[msg.name.index(name)])
        for i in range(1, 8):
            name = f'openarmx_right_joint{i}'
            if name in msg.name:
                right.append(msg.position[msg.name.index(name)])
        if len(left) == 7:
            self.left_current = np.array(left)
            self.left_ready = True
        if len(right) == 7:
            self.right_current = np.array(right)
            self.right_ready = True

    def _left_cb(self, msg: JointState):
        self._process('left', msg)

    def _right_cb(self, msg: JointState):
        self._process('right', msg)

    def _process(self, side: str, msg: JointState):
        if len(msg.position) < 7:
            return
        target = np.array(msg.position[:7], dtype=float)
        gripper = float(msg.position[7]) if len(msg.position) >= 8 else 0.0
        current = self.left_current if side == 'left' else self.right_current
        if current is None:
            if side == 'left' and not self.left_no_state_warned:
                self.get_logger().warn('[LEFT] waiting for /joint_states before forwarding arm commands')
                self.left_no_state_warned = True
            elif side == 'right' and not self.right_no_state_warned:
                self.get_logger().warn('[RIGHT] waiting for /joint_states before forwarding arm commands')
                self.right_no_state_warned = True
            return

        first = (side == 'left' and self.left_first_command) or (side == 'right' and self.right_first_command)
        if side == 'left':
            self.left_first_command = False
        else:
            self.right_first_command = False
        if first and self.enable_safety_check:
            diffs = np.abs(target - current)
            max_diff = float(np.max(diffs))
            if max_diff > self.max_joint_diff:
                self._log_safety_failure(side, current, target, diffs)
                raise SystemExit('safety check failed')
            if side == 'left':
                self.left_interp = True
                self.left_interp_start = current.copy()
                self.left_interp_target = target.copy()
                self.left_interp_time = time.time()
            else:
                self.right_interp = True
                self.right_interp_start = current.copy()
                self.right_interp_target = target.copy()
                self.right_interp_time = time.time()
        elif side == 'left':
            self.left_target = target
            if not self.left_interp:
                self._publish(side, target, gripper)
        else:
            self.right_target = target
            if not self.right_interp:
                self._publish(side, target, gripper)

    def _tick(self):
        now = time.time()
        if self.left_interp:
            p = min((now - self.left_interp_time) / self.interpolation_duration, 1.0)
            cmd = self.left_interp_start + (self.left_interp_target - self.left_interp_start) * p
            self._publish('left', cmd, 0.0)
            if p >= 1.0:
                self.left_interp = False
        if self.right_interp:
            p = min((now - self.right_interp_time) / self.interpolation_duration, 1.0)
            cmd = self.right_interp_start + (self.right_interp_target - self.right_interp_start) * p
            self._publish('right', cmd, 0.0)
            if p >= 1.0:
                self.right_interp = False

    def _log_safety_failure(self, side: str, current, target, diffs):
        names = self.LEFT_JOINT_NAMES if side == 'left' else self.RIGHT_JOINT_NAMES
        max_idx = int(np.argmax(diffs))
        lines = [
            f'[{side}] safety check failed: max joint diff '
            f'{diffs[max_idx]:.3f} rad ({np.rad2deg(diffs[max_idx]):.1f} deg) '
            f'on {names[max_idx]}, threshold={self.max_joint_diff:.3f} rad '
            f'({np.rad2deg(self.max_joint_diff):.1f} deg)',
            f'[{side}] current(deg): {np.rad2deg(current).round(1).tolist()}',
            f'[{side}] target(deg):  {np.rad2deg(target).round(1).tolist()}',
            f'[{side}] diff(deg):    {np.rad2deg(diffs).round(1).tolist()}',
        ]
        for i, name in enumerate(names):
            lines.append(
                f'[{side}] {name}: current={current[i]: .4f} rad '
                f'({np.rad2deg(current[i]): .1f} deg), target={target[i]: .4f} rad '
                f'({np.rad2deg(target[i]): .1f} deg), diff={diffs[i]: .4f} rad '
                f'({np.rad2deg(diffs[i]): .1f} deg)'
            )
        self.get_logger().error('\n' + '\n'.join(lines))

    def _publish(self, side: str, joint_positions, gripper: float):
        msg = Float64MultiArray()
        msg.data = list(joint_positions) + [float(gripper) * self.gripper_scale]
        if side == 'left':
            self.left_pub.publish(msg)
        else:
            self.right_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpenFlexArmBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

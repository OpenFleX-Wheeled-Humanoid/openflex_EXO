#!/usr/bin/env python3
import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float32, Float64, Float64MultiArray


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def apply_deadzone(value, deadzone):
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0.0 else -1.0
    return sign * (abs(value) - deadzone) / max(1.0 - deadzone, 1e-6)


@dataclass
class SCurveState:
    current: float = 0.0
    accel: float = 0.0


class SCurveController:
    def __init__(self, max_velocity, acceleration_time, smoothness, dt):
        self.max_velocity = float(max_velocity)
        self.acceleration_time = max(0.1, float(acceleration_time))
        self.smoothness = clamp(float(smoothness), 0.05, 0.95)
        self.dt = float(dt)
        self.state = SCurveState()
        self._update_params()

    def _update_params(self):
        self.max_acceleration = self.max_velocity / self.acceleration_time
        jerk_time = max(self.acceleration_time * self.smoothness, 0.01)
        self.max_jerk = self.max_acceleration / jerk_time

    def update_params(self, max_velocity=None):
        if max_velocity is not None:
            self.max_velocity = float(max_velocity)
        self._update_params()

    def update(self, target):
        target = clamp(target, -self.max_velocity, self.max_velocity)
        error = target - self.state.current
        desired_acc = clamp(error / self.dt, -self.max_acceleration, self.max_acceleration)
        acc_change = desired_acc - self.state.accel
        max_change = self.max_jerk * self.dt
        acc_change = clamp(acc_change, -max_change, max_change)
        self.state.accel = clamp(self.state.accel + acc_change,
                                 -self.max_acceleration, self.max_acceleration)
        self.state.current = clamp(self.state.current + self.state.accel * self.dt,
                                   -self.max_velocity, self.max_velocity)
        return self.state.current

    def reset(self):
        self.state = SCurveState()


class OpenFlexExoTeleopNode(Node):
    def __init__(self):
        super().__init__('openflex_exo_teleop_node')
        self.declare_parameter('exo_gamepad_topic', '/exo/gamepad_keys')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('lift_jog_topic', '/lift_manual_position_controller/jog_command')
        self.declare_parameter('head_command_topic', '/head_forward_position_controller/commands')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('speed_scale_topic', '/openflex_teleop_exo/speed_scale')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('input_timeout_sec', 0.5)
        self.declare_parameter('joystick_deadzone', 0.15)
        self.declare_parameter('linear_expo', 1.0)
        self.declare_parameter('angular_expo', 1.0)
        self.declare_parameter('enable_scurve', True)
        self.declare_parameter('acceleration_time', 0.5)
        self.declare_parameter('smoothness', 0.3)
        self.declare_parameter('base_max_linear_speed', 1.0)
        self.declare_parameter('base_max_angular_speed', 2.0)
        self.declare_parameter('base_lift_speed', 0.05)
        self.declare_parameter('base_head_rate_deg_s', 30.0)
        self.declare_parameter('default_speed_scale', 0.1)
        self.declare_parameter('min_speed_scale', 0.0)
        self.declare_parameter('max_speed_scale', 1.0)
        self.declare_parameter('axis_left_x', 0)
        self.declare_parameter('axis_left_y', 1)
        self.declare_parameter('axis_right_x', 2)
        self.declare_parameter('chassis_vx_sign', 1.0)
        self.declare_parameter('chassis_vy_sign', 1.0)
        self.declare_parameter('chassis_wz_sign', 1.0)
        self.declare_parameter('buttons_active_low', True)
        self.declare_parameter('button_left_red', 5)
        self.declare_parameter('button_left_blue', 6)
        self.declare_parameter('button_right_red', 11)
        self.declare_parameter('button_right_blue', 12)
        self.declare_parameter('button_right_white', 13)
        self.declare_parameter('button_right_black', 14)
        self.declare_parameter('lift_down_sign', -1.0)
        self.declare_parameter('lift_up_sign', 1.0)
        self.declare_parameter('yaw_joint_name', 'openarmx_head_yaw_joint')
        self.declare_parameter('pitch_joint_name', 'openarmx_head_pitch_joint')
        self.declare_parameter('head_yaw_left_sign', 1.0)
        self.declare_parameter('head_yaw_right_sign', -1.0)
        self.declare_parameter('head_pitch_up_sign', 1.0)
        self.declare_parameter('head_pitch_down_sign', -1.0)
        self.declare_parameter('invert_head_yaw', False)
        self.declare_parameter('invert_head_pitch', False)
        self.declare_parameter('head_limit_rad', 1.5708)
        self.declare_parameter('head_soft_margin_deg', 10.0)
        self.declare_parameter('publish_head_hold_on_start', False)

        self.gamepad_topic = self.get_parameter('exo_gamepad_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.lift_topic = self.get_parameter('lift_jog_topic').value
        self.head_topic = self.get_parameter('head_command_topic').value
        self.joint_states_topic = self.get_parameter('joint_states_topic').value
        self.speed_scale_topic = self.get_parameter('speed_scale_topic').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.input_timeout_sec = float(self.get_parameter('input_timeout_sec').value)
        self.deadzone = float(self.get_parameter('joystick_deadzone').value)
        self.linear_expo = max(1.0, float(self.get_parameter('linear_expo').value))
        self.angular_expo = max(1.0, float(self.get_parameter('angular_expo').value))
        self.enable_scurve = bool(self.get_parameter('enable_scurve').value)
        self.acceleration_time = float(self.get_parameter('acceleration_time').value)
        self.smoothness = float(self.get_parameter('smoothness').value)
        self.base_max_linear_speed = float(self.get_parameter('base_max_linear_speed').value)
        self.base_max_angular_speed = float(self.get_parameter('base_max_angular_speed').value)
        self.base_lift_speed = float(self.get_parameter('base_lift_speed').value)
        self.base_head_rate_deg_s = float(self.get_parameter('base_head_rate_deg_s').value)
        self.speed_scale = clamp(float(self.get_parameter('default_speed_scale').value),
                                 float(self.get_parameter('min_speed_scale').value),
                                 float(self.get_parameter('max_speed_scale').value))
        self.min_speed_scale = float(self.get_parameter('min_speed_scale').value)
        self.max_speed_scale = float(self.get_parameter('max_speed_scale').value)
        self.axis_left_x = int(self.get_parameter('axis_left_x').value)
        self.axis_left_y = int(self.get_parameter('axis_left_y').value)
        self.axis_right_x = int(self.get_parameter('axis_right_x').value)
        self.chassis_vx_sign = float(self.get_parameter('chassis_vx_sign').value)
        self.chassis_vy_sign = float(self.get_parameter('chassis_vy_sign').value)
        self.chassis_wz_sign = float(self.get_parameter('chassis_wz_sign').value)
        self.buttons_active_low = bool(self.get_parameter('buttons_active_low').value)
        self.button_left_red = int(self.get_parameter('button_left_red').value)
        self.button_left_blue = int(self.get_parameter('button_left_blue').value)
        self.button_right_red = int(self.get_parameter('button_right_red').value)
        self.button_right_blue = int(self.get_parameter('button_right_blue').value)
        self.button_right_white = int(self.get_parameter('button_right_white').value)
        self.button_right_black = int(self.get_parameter('button_right_black').value)
        self.lift_down_sign = float(self.get_parameter('lift_down_sign').value)
        self.lift_up_sign = float(self.get_parameter('lift_up_sign').value)
        self.yaw_joint_name = self.get_parameter('yaw_joint_name').value
        self.pitch_joint_name = self.get_parameter('pitch_joint_name').value
        self.head_yaw_left_sign = float(self.get_parameter('head_yaw_left_sign').value)
        self.head_yaw_right_sign = float(self.get_parameter('head_yaw_right_sign').value)
        self.head_pitch_up_sign = float(self.get_parameter('head_pitch_up_sign').value)
        self.head_pitch_down_sign = float(self.get_parameter('head_pitch_down_sign').value)
        self.invert_head_yaw = bool(self.get_parameter('invert_head_yaw').value)
        self.invert_head_pitch = bool(self.get_parameter('invert_head_pitch').value)
        self.head_limit_rad = float(self.get_parameter('head_limit_rad').value)
        self.head_soft_margin = math.radians(float(self.get_parameter('head_soft_margin_deg').value))
        self.head_command_limit = max(0.0, self.head_limit_rad - self.head_soft_margin)
        self.publish_head_hold_on_start = bool(self.get_parameter('publish_head_hold_on_start').value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.lift_pub = self.create_publisher(Float64, self.lift_topic, 10)
        self.head_pub = self.create_publisher(Float64MultiArray, self.head_topic, 10)
        self.speed_pub = self.create_publisher(Float32, self.speed_scale_topic, 10)

        self.create_subscription(Joy, self.gamepad_topic, self._joy_cb, 10)
        self.create_subscription(JointState, self.joint_states_topic, self._joint_states_cb, 10)
        self.create_subscription(Float32, self.speed_scale_topic, self._speed_scale_cb, 10)

        self.latest_axes = [0.0, 0.0, 0.0, 0.0]
        self.latest_buttons = []
        self.have_input = False
        self.last_input_time = self.get_clock().now()
        self.last_speed_scale_publish = self.speed_scale
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        self.have_joint_states = False
        self.last_left_red = None
        self.last_left_blue = None
        self.last_right_red = None
        self.last_right_blue = None
        self.last_right_white = None
        self.last_right_black = None
        self.lift_active_direction = 0

        dt = 1.0 / max(self.publish_rate, 1.0)
        self.scurve_vx = SCurveController(self.base_max_linear_speed, self.acceleration_time,
                                          self.smoothness, dt) if self.enable_scurve else None
        self.scurve_vy = SCurveController(self.base_max_linear_speed, self.acceleration_time,
                                          self.smoothness, dt) if self.enable_scurve else None
        self.scurve_wz = SCurveController(self.base_max_angular_speed, self.acceleration_time,
                                          self.smoothness, dt) if self.enable_scurve else None

        self.head_target_yaw = 0.0
        self.head_target_pitch = 0.0
        self.head_target_initialized = False

        self.create_timer(1.0 / max(self.publish_rate, 1.0), self._control_step)
        self.create_timer(0.1, self._publish_speed_scale)

        self.get_logger().info('openflex_exo_teleop_node started')
        self.get_logger().info('chassis=/cmd_vel lift=/lift_manual_position_controller/jog_command head=/head_forward_position_controller/commands')

        if self.publish_head_hold_on_start:
            self._publish_head_hold()

    def _normalize_button(self, value):
        if self.buttons_active_low:
            return int(value) == 0
        return int(value) != 0

    def _joy_cb(self, msg: Joy):
        self.latest_axes = list(msg.axes)
        self.latest_buttons = list(msg.buttons)
        self.have_input = True
        self.last_input_time = self.get_clock().now()

    def _speed_scale_cb(self, msg: Float32):
        self.speed_scale = clamp(float(msg.data), self.min_speed_scale, self.max_speed_scale)

    def _joint_states_cb(self, msg: JointState):
        try:
            yaw_idx = msg.name.index(self.yaw_joint_name)
            pitch_idx = msg.name.index(self.pitch_joint_name)
        except ValueError:
            return
        self.current_yaw = float(msg.position[yaw_idx])
        self.current_pitch = float(msg.position[pitch_idx])
        self.have_joint_states = True
        if not self.head_target_initialized:
            self.head_target_yaw = clamp(self.current_yaw, -self.head_command_limit, self.head_command_limit)
            self.head_target_pitch = clamp(self.current_pitch, -self.head_command_limit, self.head_command_limit)
            self.head_target_initialized = True

    def _publish_speed_scale(self):
        msg = Float32()
        msg.data = float(self.speed_scale)
        self.speed_pub.publish(msg)

    def _scaled_speed(self, base):
        return base * max(self.speed_scale, self.min_speed_scale)

    def _publish_chassis(self):
        left_x = apply_deadzone(self.latest_axes[self.axis_left_x] if len(self.latest_axes) > self.axis_left_x else 0.0,
                                self.deadzone)
        left_y = apply_deadzone(self.latest_axes[self.axis_left_y] if len(self.latest_axes) > self.axis_left_y else 0.0,
                                self.deadzone)
        right_x = apply_deadzone(self.latest_axes[self.axis_right_x] if len(self.latest_axes) > self.axis_right_x else 0.0,
                                 self.deadzone)

        left_x = math.copysign(abs(left_x) ** self.linear_expo, left_x)
        left_y = math.copysign(abs(left_y) ** self.linear_expo, left_y)
        right_x = math.copysign(abs(right_x) ** self.angular_expo, right_x)

        target_vx = self.chassis_vx_sign * left_y * self._scaled_speed(self.base_max_linear_speed)
        target_vy = self.chassis_vy_sign * left_x * self._scaled_speed(self.base_max_linear_speed)
        target_wz = self.chassis_wz_sign * right_x * self._scaled_speed(self.base_max_angular_speed)

        if self.enable_scurve:
            target_vx = self.scurve_vx.update(target_vx)
            target_vy = self.scurve_vy.update(target_vy)
            target_wz = self.scurve_wz.update(target_wz)

        msg = Twist()
        msg.linear.x = target_vx
        msg.linear.y = target_vy
        msg.angular.z = target_wz
        self.cmd_pub.publish(msg)

    def _publish_lift(self):
        if len(self.latest_buttons) <= max(self.button_left_red, self.button_left_blue):
            return
        red = self._normalize_button(self.latest_buttons[self.button_left_red])
        blue = self._normalize_button(self.latest_buttons[self.button_left_blue])

        direction = 0
        if blue and not red:
            direction = 1
        elif red and not blue:
            direction = -1

        if direction == 0:
            if self.lift_active_direction != 0:
                msg = Float64()
                msg.data = 0.0
                self.lift_pub.publish(msg)
            self.lift_active_direction = 0
            return

        self.lift_active_direction = direction
        msg = Float64()
        if direction > 0:
            msg.data = self.lift_up_sign * self._scaled_speed(self.base_lift_speed)
        else:
            msg.data = self.lift_down_sign * self._scaled_speed(self.base_lift_speed)
        self.lift_pub.publish(msg)

    def _publish_head_hold(self):
        msg = Float64MultiArray()
        msg.data = [float(self.current_yaw), float(self.current_pitch)]
        self.head_pub.publish(msg)

    def _publish_head(self):
        if len(self.latest_buttons) <= max(self.button_right_red, self.button_right_blue,
                                          self.button_right_white, self.button_right_black):
            return

        red = self._normalize_button(self.latest_buttons[self.button_right_red])
        blue = self._normalize_button(self.latest_buttons[self.button_right_blue])
        white = self._normalize_button(self.latest_buttons[self.button_right_white])
        black = self._normalize_button(self.latest_buttons[self.button_right_black])

        yaw_delta = 0.0
        pitch_delta = 0.0
        step = math.radians(self.base_head_rate_deg_s / max(self.publish_rate, 1.0)) * max(self.speed_scale, self.min_speed_scale)

        if red:
            pitch_delta += self.head_pitch_down_sign * step
        if blue:
            pitch_delta += self.head_pitch_up_sign * step
        if white:
            yaw_delta += self.head_yaw_left_sign * step
        if black:
            yaw_delta += self.head_yaw_right_sign * step

        if self.invert_head_yaw:
            yaw_delta = -yaw_delta
        if self.invert_head_pitch:
            pitch_delta = -pitch_delta

        if abs(yaw_delta) < 1e-12 and abs(pitch_delta) < 1e-12:
            return

        if not self.head_target_initialized:
            self.head_target_yaw = clamp(self.current_yaw, -self.head_command_limit, self.head_command_limit)
            self.head_target_pitch = clamp(self.current_pitch, -self.head_command_limit, self.head_command_limit)
            self.head_target_initialized = True

        self.head_target_yaw = clamp(self.head_target_yaw + yaw_delta,
                                     -self.head_command_limit, self.head_command_limit)
        self.head_target_pitch = clamp(self.head_target_pitch + pitch_delta,
                                       -self.head_command_limit, self.head_command_limit)

        msg = Float64MultiArray()
        msg.data = [float(self.head_target_yaw), float(self.head_target_pitch)]
        self.head_pub.publish(msg)

    def _control_step(self):
        if not self.have_input:
            return
        if (self.get_clock().now() - self.last_input_time).nanoseconds / 1e9 > self.input_timeout_sec:
            stop = Twist()
            self.cmd_pub.publish(stop)
            self.lift_pub.publish(Float64(data=0.0))
            if self.have_joint_states:
                self._publish_head_hold()
            return
        self._publish_chassis()
        self._publish_lift()
        if self.have_joint_states:
            self._publish_head()


def main(args=None):
    rclpy.init(args=args)
    node = OpenFlexExoTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import asyncio
import json
import threading
import time
from typing import Dict

import numpy as np
import rclpy
import websockets
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Header

from .protocol import ExoProtocolParser


class WebSocketTeleoperator(Node):
    def __init__(self):
        super().__init__('websocket_teleoperator')
        self.declare_parameter('websocket_host', '0.0.0.0')
        self.declare_parameter('websocket_port', 19091)
        self.declare_parameter('joint_command_topic', '/exo/joint_command')
        self.declare_parameter('gamepad_keys_topic', '/exo/gamepad_keys')
        self.declare_parameter('enable_left_arm', True)
        self.declare_parameter('enable_right_arm', True)
        self.declare_parameter('enable_left_joystick', True)
        self.declare_parameter('enable_right_joystick', True)
        self.declare_parameter('enable_vehicle_control', True)
        self.declare_parameter('max_publish_rate_hz', 100.0)
        self.declare_parameter('require_switches_up', True)

        self.websocket_host = self.get_parameter('websocket_host').value
        self.websocket_port = int(self.get_parameter('websocket_port').value)
        self.joint_command_topic = self.get_parameter('joint_command_topic').value
        self.gamepad_keys_topic = self.get_parameter('gamepad_keys_topic').value
        self.enable_left_arm = bool(self.get_parameter('enable_left_arm').value)
        self.enable_right_arm = bool(self.get_parameter('enable_right_arm').value)
        self.enable_left_joystick = bool(self.get_parameter('enable_left_joystick').value)
        self.enable_right_joystick = bool(self.get_parameter('enable_right_joystick').value)
        self.enable_vehicle_control = bool(self.get_parameter('enable_vehicle_control').value)
        self.require_switches_up = bool(self.get_parameter('require_switches_up').value)
        self.max_publish_rate_hz = float(self.get_parameter('max_publish_rate_hz').value)
        self.min_publish_interval = 1.0 / max(self.max_publish_rate_hz, 1.0)

        self.arms_control_enabled = True
        self.joystick_control_enabled = True
        self.homing_active = False
        self.left_button_last_state = 1
        self.right_button_last_state = 1
        self.left_button_pressed = False
        self.right_button_pressed = False
        self.left_button_press_start_time = None
        self.right_button_press_start_time = None
        self.left_button_press_start_time_saved = None
        self.right_button_press_start_time_saved = None
        self.left_button_release_time = None
        self.right_button_release_time = None
        self.control_toggle_duration = 1.0
        self.homing_duration = 10.0
        self.debounce_duration = 0.05
        self.dual_release_window = 0.2
        self.left_switch_last_state = None
        self.right_switch_last_state = None
        self.both_switches_enabled_last = False

        self.joint_names = [
            'left_arm_joint_1', 'left_arm_joint_2', 'left_arm_joint_3',
            'left_arm_joint_4', 'left_arm_joint_5', 'left_arm_joint_6', 'left_arm_joint_7',
            'right_arm_joint_1', 'right_arm_joint_2', 'right_arm_joint_3',
            'right_arm_joint_4', 'right_arm_joint_5', 'right_arm_joint_6', 'right_arm_joint_7',
            'left_trigger_joint', 'right_trigger_joint',
        ]
        self.default_joint_positions = [0.0] * 16
        self.default_velocity = [0.0] * 16
        self.default_effort = [0.0] * 16
        self.left_arm_joint_indices = list(range(0, 7))
        self.right_arm_joint_indices = list(range(7, 14))
        self.left_trigger_index = 14
        self.right_trigger_index = 15

        self.parser = ExoProtocolParser()
        self.joint_command_pub = self.create_publisher(JointState, self.joint_command_topic, 10)
        self.gamepad_keys_pub = self.create_publisher(Joy, self.gamepad_keys_topic, 10)

        self.websocket_server = None
        self.connected_clients = set()
        self.websocket_running = False
        self.last_publish_time = 0.0
        self.stats = {
            'total_received': 0,
            'total_published': 0,
            'parse_errors': 0,
            'dropped_by_rate_limit': 0,
            'dropped_by_switch_check': 0,
            'last_switch_warning_time': 0.0,
        }

        self.start_websocket_server()
        self.get_logger().info(
            f'OpenFlex WebSocket EXO publisher started'
            f'\n  WebSocket: {self.websocket_host}:{self.websocket_port}'
            f'\n  Joint command topic: {self.joint_command_topic}'
            f'\n  Gamepad topic: {self.gamepad_keys_topic}'
            f'\n  Max publish rate: {self.max_publish_rate_hz:.1f} Hz'
            f'\n  Switch safety: {"enabled" if self.require_switches_up else "disabled"}'
        )

    def start_websocket_server(self):
        def run_server():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                self.websocket_running = True

                async def server_main():
                    server = await websockets.serve(
                        self.handle_websocket_connection,
                        self.websocket_host,
                        self.websocket_port,
                    )
                    self.websocket_server = server
                    self.get_logger().info(
                        f'WebSocket server listening on {self.websocket_host}:{self.websocket_port}')
                    try:
                        while self.websocket_running:
                            await asyncio.sleep(0.1)
                    finally:
                        server.close()
                        await server.wait_closed()

                loop.run_until_complete(server_main())
            except Exception as exc:
                self.get_logger().error(f'WebSocket server failed: {exc}')
            finally:
                self.websocket_running = False
                pending = asyncio.all_tasks(loop)
                for task in pending:
                    task.cancel()
                if pending:
                    loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
                loop.close()

        self.websocket_thread = threading.Thread(target=run_server, daemon=True)
        self.websocket_thread.start()

    async def handle_websocket_connection(self, websocket):
        client_address = websocket.remote_address
        self.connected_clients.add(websocket)
        self.get_logger().info(f'EXO WebSocket client connected: {client_address}')
        try:
            async for message in websocket:
                await self.process_websocket_message(message)
        except websockets.exceptions.ConnectionClosed as exc:
            self.get_logger().info(f'EXO WebSocket client disconnected: {client_address} - {exc}')
        except Exception as exc:
            self.get_logger().error(f'EXO WebSocket client error {client_address}: {exc}')
        finally:
            self.connected_clients.discard(websocket)

    async def process_websocket_message(self, message: str):
        try:
            data = json.loads(message)
            parsed_data = self.parser.parse_websocket_message(data)
            if not parsed_data or not parsed_data.get('valid', False):
                self.stats['parse_errors'] += 1
                return
            self.detect_button_clicks(parsed_data)
            if not self.arms_control_enabled and not self.joystick_control_enabled:
                self.stats['total_received'] += 1
                return
            self.process_and_publish_data(parsed_data)
            self.stats['total_received'] += 1
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Invalid EXO JSON message: {exc}')
        except Exception as exc:
            self.get_logger().error(f'Failed to process EXO WebSocket message: {exc}')

    def process_and_publish_data(self, parsed_data: Dict):
        current_time = time.time()
        if current_time - self.last_publish_time < self.min_publish_interval:
            self.stats['dropped_by_rate_limit'] += 1
            return

        left_joystick = parsed_data.get('left_joystick', {})
        right_joystick = parsed_data.get('right_joystick', {})
        left_buttons = left_joystick.get('buttons', {})
        right_buttons = right_joystick.get('buttons', {})

        if not self._switches_allow_publish(left_buttons, right_buttons):
            return

        joint_positions = self.default_joint_positions.copy()
        if self.enable_left_arm and self.arms_control_enabled:
            for i, value in enumerate(parsed_data.get('left_arm_joints', [])[:7]):
                joint_positions[self.left_arm_joint_indices[i]] = float(value)
        if self.enable_right_arm and self.arms_control_enabled:
            for i, value in enumerate(parsed_data.get('right_arm_joints', [])[:7]):
                joint_positions[self.right_arm_joint_indices[i]] = float(value)

        if self.enable_left_joystick and self.joystick_control_enabled:
            joint_positions[self.left_trigger_index] = self.map_trigger_to_gripper(
                left_joystick.get('trigger', 1.0))
        if self.enable_right_joystick and self.joystick_control_enabled:
            joint_positions[self.right_trigger_index] = self.map_trigger_to_gripper(
                right_joystick.get('trigger', 1.0))

        joystick_data = {
            'left_x': float(left_joystick.get('x', 0.0)) if self.enable_left_joystick else 0.0,
            'left_y': float(left_joystick.get('y', 0.0)) if self.enable_left_joystick else 0.0,
            'right_x': float(right_joystick.get('x', 0.0)) if self.enable_right_joystick else 0.0,
            'right_y': float(right_joystick.get('y', 0.0)) if self.enable_right_joystick else 0.0,
            'left_buttons': left_buttons if self.enable_left_joystick else {},
            'right_buttons': right_buttons if self.enable_right_joystick else {},
        }
        self.publish_joint_command(joint_positions)
        self.publish_gamepad_keys(joystick_data)
        self.last_publish_time = current_time
        self.stats['total_published'] += 1

    def _switches_allow_publish(self, left_buttons: Dict, right_buttons: Dict) -> bool:
        if not self.require_switches_up:
            return True
        left_switch = left_buttons.get('switch', 0) if self.enable_left_joystick else 0
        right_switch = right_buttons.get('switch', 0) if self.enable_right_joystick else 0
        both_up = (
            (not self.enable_left_joystick or left_switch == 0) and
            (not self.enable_right_joystick or right_switch == 0)
        )
        if both_up != self.both_switches_enabled_last:
            if both_up:
                self.get_logger().info('Both EXO handle switches are up; forwarding data')
            else:
                self.get_logger().warn(
                    f'EXO handle switches are not both up; left={left_switch}, right={right_switch}')
            self.both_switches_enabled_last = both_up
        if not both_up:
            self.stats['dropped_by_switch_check'] += 1
            now = time.time()
            if now - self.stats['last_switch_warning_time'] >= 5.0:
                self.get_logger().warn(
                    f'Dropped EXO data because switches are not both up '
                    f'(left={left_switch}, right={right_switch})')
                self.stats['last_switch_warning_time'] = now
            return False
        return True

    def map_trigger_to_gripper(self, trigger_value):
        return float(np.clip(trigger_value, 0.0, 1.0) * 0.067)

    def detect_button_clicks(self, exo_data):
        current_time = time.time()
        left_joystick = exo_data.get('left_joystick', {})
        right_joystick = exo_data.get('right_joystick', {})
        left_button_current = left_joystick.get('button', 1)
        right_button_current = right_joystick.get('button', 1)

        if self.left_button_last_state == 1 and left_button_current == 0:
            self.left_button_pressed = True
            self.left_button_press_start_time = current_time
        elif self.left_button_last_state == 0 and left_button_current == 1:
            self.left_button_release_time = current_time
            self.left_button_press_start_time_saved = self.left_button_press_start_time
            self.left_button_pressed = False
            self.left_button_press_start_time = None

        if self.right_button_last_state == 1 and right_button_current == 0:
            self.right_button_pressed = True
            self.right_button_press_start_time = current_time
        elif self.right_button_last_state == 0 and right_button_current == 1:
            self.right_button_release_time = current_time
            self.right_button_press_start_time_saved = self.right_button_press_start_time
            self.right_button_pressed = False
            self.right_button_press_start_time = None

        self.check_dual_release_with_tolerance(current_time)
        self.left_button_last_state = left_button_current
        self.right_button_last_state = right_button_current

    def check_dual_release_with_tolerance(self, current_time):
        if self.left_button_release_time is None or self.right_button_release_time is None:
            return
        time_diff = abs(self.left_button_release_time - self.right_button_release_time)
        if time_diff > self.dual_release_window:
            return
        latest_release_time = max(self.left_button_release_time, self.right_button_release_time)
        if abs(current_time - latest_release_time) >= 0.05:
            return
        self.check_dual_long_press_release(latest_release_time)
        self.left_button_release_time = None
        self.right_button_release_time = None

    def check_dual_long_press_release(self, current_time):
        if self.left_button_press_start_time_saved is None or self.right_button_press_start_time_saved is None:
            return
        min_press_start_time = max(
            self.left_button_press_start_time_saved,
            self.right_button_press_start_time_saved,
        )
        press_duration = current_time - min_press_start_time
        if press_duration >= self.homing_duration:
            self.get_logger().warn('Homing by EXO button hold is disabled; use safe robot-side homing')
        elif press_duration >= self.control_toggle_duration:
            self.toggle_control_state()
            self.get_logger().info(
                f'EXO control toggled: arms={self.arms_control_enabled}, '
                f'joystick={self.joystick_control_enabled}')
        self.left_button_press_start_time_saved = None
        self.right_button_press_start_time_saved = None

    def toggle_control_state(self):
        if self.enable_left_arm and self.enable_right_arm:
            self.arms_control_enabled = not self.arms_control_enabled
        if self.enable_left_joystick and self.enable_right_joystick and self.enable_vehicle_control:
            self.joystick_control_enabled = not self.joystick_control_enabled

    def publish_joint_command(self, positions):
        joint_cmd = JointState()
        joint_cmd.header = Header()
        joint_cmd.header.stamp = self.get_clock().now().to_msg()
        joint_cmd.name = self.joint_names
        joint_cmd.position = [float(p) for p in positions]
        joint_cmd.velocity = list(self.default_velocity)
        joint_cmd.effort = list(self.default_effort)
        self.joint_command_pub.publish(joint_cmd)

    def publish_gamepad_keys(self, joystick_data):
        left_buttons = joystick_data.get('left_buttons', {})
        right_buttons = joystick_data.get('right_buttons', {})
        gamepad_msg = Joy()
        gamepad_msg.header = Header()
        gamepad_msg.header.stamp = self.get_clock().now().to_msg()
        gamepad_msg.axes = [
            float(joystick_data.get('left_x', 0.0)),
            float(joystick_data.get('left_y', 0.0)),
            float(joystick_data.get('right_x', 0.0)),
            float(joystick_data.get('right_y', 0.0)),
        ]
        gamepad_msg.buttons = [
            1 if self.arms_control_enabled else 0,
            1 if self.joystick_control_enabled else 0,
            1 if self.homing_active else 0,
            1 if self.enable_vehicle_control else 0,
            left_buttons.get('joystick_k', 0),
            left_buttons.get('button_a', 0),
            left_buttons.get('button_b', 0),
            left_buttons.get('button_c', 0),
            left_buttons.get('button_d', 0),
            left_buttons.get('switch', 0),
            right_buttons.get('joystick_k', 0),
            right_buttons.get('button_a', 0),
            right_buttons.get('button_b', 0),
            right_buttons.get('button_c', 0),
            right_buttons.get('button_d', 0),
            right_buttons.get('switch', 0),
        ]
        self.gamepad_keys_pub.publish(gamepad_msg)

    def destroy_node(self):
        self.get_logger().info('Shutting down OpenFlex EXO WebSocket publisher')
        self.websocket_running = False
        if hasattr(self, 'websocket_thread') and self.websocket_thread.is_alive():
            self.websocket_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebSocketTeleoperator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

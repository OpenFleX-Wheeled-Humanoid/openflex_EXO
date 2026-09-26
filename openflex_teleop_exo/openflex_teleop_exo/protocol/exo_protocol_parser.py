#!/usr/bin/env python3
import struct
from typing import Dict, Optional, Tuple

import numpy as np


class ExoProtocolParser:
    FRAME_HEADER = 0xAA
    FRAME_TAIL = 0x55

    DATA_LEN_BASE = 48
    DATA_LEN_TORSO_IMU_QUAT = 88
    DATA_LEN_TORSO_AND_EXTRA_IMU_QUAT = 128
    SUPPORTED_DATA_LENGTHS = [
        DATA_LEN_BASE,
        DATA_LEN_TORSO_IMU_QUAT,
        DATA_LEN_TORSO_AND_EXTRA_IMU_QUAT,
    ]

    ENCODER_TO_RADIAN = (2 * np.pi) / 16384

    JOYSTICK_LEFT_OFFSET = 0
    JOYSTICK_RIGHT_OFFSET = 8
    ARM_JOINT_LEFT_OFFSET = 16
    ARM_JOINT_RIGHT_OFFSET = 32

    JOYSTICK_MIN = 200
    JOYSTICK_MAX = 3896
    JOYSTICK_CENTER = 2048
    JOYSTICK_DEADZONE = 200

    TRIGGER_MIN = 700
    TRIGGER_MAX = 3700

    def __init__(self):
        self.last_parsed_data = None
        self.parse_count = 0

    def calculate_checksum(self, data: bytes) -> int:
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    def validate_frame(self, data: bytes, data_len: int) -> bool:
        frame_size = 1 + data_len + 2
        if len(data) < frame_size:
            return False
        if data[0] != self.FRAME_HEADER:
            return False
        if data[frame_size - 1] != self.FRAME_TAIL:
            return False
        payload = data[1:frame_size - 2]
        return self.calculate_checksum(payload) == data[frame_size - 2]

    def parse_websocket_message(self, message: Dict) -> Optional[Dict]:
        try:
            if not isinstance(message, dict):
                return None
            if message.get('type') != 'exoskeleton-realtime':
                return None
            data_section = message.get('data', {})
            if not data_section or data_section.get('type') != 'binary':
                return None
            raw_data = data_section.get('data', [])
            if not raw_data:
                return None

            data_bytes = bytes(raw_data)
            parsed_data = None
            for data_len in self.SUPPORTED_DATA_LENGTHS:
                frame_size = 1 + data_len + 2
                if len(data_bytes) != frame_size:
                    continue
                if not self.validate_frame(data_bytes, data_len):
                    continue
                parsed_data = self._parse_frame_data(data_bytes, data_len)
                if parsed_data:
                    break

            if not parsed_data:
                return None

            parsed_data['timestamp'] = message.get('timestamp', 0)
            parsed_data['device_id'] = message.get('deviceId', 'unknown')
            parsed_data['valid'] = True
            self.last_parsed_data = parsed_data
            self.parse_count += 1
            return parsed_data
        except Exception as exc:
            print(f'解析 WebSocket 消息失败: {exc}')
            return None

    def _parse_frame_data(self, data_bytes: bytes, data_len: int) -> Optional[Dict]:
        try:
            frame_size = 1 + data_len + 2
            payload = data_bytes[1:frame_size - 2]

            left_joystick_raw = struct.unpack(
                '<4h', payload[self.JOYSTICK_LEFT_OFFSET:self.JOYSTICK_LEFT_OFFSET + 8])
            right_joystick_raw = struct.unpack(
                '<4h', payload[self.JOYSTICK_RIGHT_OFFSET:self.JOYSTICK_RIGHT_OFFSET + 8])
            left_arm_raw = struct.unpack(
                '<8h', payload[self.ARM_JOINT_LEFT_OFFSET:self.ARM_JOINT_LEFT_OFFSET + 16])
            right_arm_raw = struct.unpack(
                '<8h', payload[self.ARM_JOINT_RIGHT_OFFSET:self.ARM_JOINT_RIGHT_OFFSET + 16])

            result = {
                'left_arm_joints': [self._encoder_to_radian(joint) for joint in left_arm_raw],
                'right_arm_joints': [self._encoder_to_radian(joint) for joint in right_arm_raw],
                'left_arm_joints_raw': list(left_arm_raw),
                'right_arm_joints_raw': list(right_arm_raw),
                'left_joystick': self._parse_joystick_data(left_joystick_raw),
                'right_joystick': self._parse_joystick_data(right_joystick_raw),
                'torso_imu': {},
                'extra_imu': {},
                'format_version': 1,
            }

            offset = 48
            if data_len == self.DATA_LEN_TORSO_IMU_QUAT:
                result['format_version'] = 2
                result['torso_imu'] = self._parse_imu(payload, offset)
            elif data_len == self.DATA_LEN_TORSO_AND_EXTRA_IMU_QUAT:
                result['format_version'] = 3
                result['torso_imu'] = self._parse_imu(payload, offset)
                result['extra_imu'] = self._parse_imu(payload, offset + 40)
            return result
        except Exception as exc:
            print(f'解析数据帧失败: {exc}')
            return None

    def _parse_imu(self, payload: bytes, offset: int) -> Dict:
        acc = struct.unpack('<3f', payload[offset:offset + 12])
        gyro = struct.unpack('<3f', payload[offset + 12:offset + 24])
        quat = struct.unpack('<4f', payload[offset + 24:offset + 40])
        return {'acc': list(acc), 'gyro': list(gyro), 'quat': list(quat)}

    def _parse_joystick_data(self, raw_data: Tuple[int, int, int, int]) -> Dict:
        x, y, button_info, trigger = raw_data
        button_byte = button_info & 0xFF
        return {
            'x': self._normalize_joystick_axis(x),
            'y': self._normalize_joystick_axis(y),
            'button': button_info,
            'buttons': self._parse_button_info(button_byte),
            'trigger': self._normalize_trigger(trigger),
        }

    def _parse_button_info(self, button_byte: int) -> Dict:
        return {
            'joystick_k': (button_byte >> 0) & 0x01,
            'button_a': (button_byte >> 1) & 0x01,
            'button_b': (button_byte >> 2) & 0x01,
            'button_c': (button_byte >> 3) & 0x01,
            'button_d': (button_byte >> 4) & 0x01,
            'switch': (button_byte >> 5) & 0x01,
        }

    def _normalize_joystick_axis(self, value: int) -> float:
        if abs(value - self.JOYSTICK_CENTER) < self.JOYSTICK_DEADZONE:
            return 0.0
        if value > self.JOYSTICK_CENTER:
            return (value - self.JOYSTICK_CENTER) / (self.JOYSTICK_MAX - self.JOYSTICK_CENTER)
        return (value - self.JOYSTICK_CENTER) / (self.JOYSTICK_CENTER - self.JOYSTICK_MIN)

    def _normalize_trigger(self, value: int) -> float:
        if value <= self.TRIGGER_MIN:
            return 1.0
        if value >= self.TRIGGER_MAX:
            return 0.0
        normalized = (value - self.TRIGGER_MIN) / (self.TRIGGER_MAX - self.TRIGGER_MIN)
        return 1.0 - normalized

    def _encoder_to_radian(self, encoder_value: int) -> float:
        return encoder_value * self.ENCODER_TO_RADIAN

    def get_statistics(self) -> Dict:
        return {
            'total_parsed': self.parse_count,
            'last_parse_time': self.last_parsed_data.get('timestamp', 0)
            if self.last_parsed_data else 0,
        }

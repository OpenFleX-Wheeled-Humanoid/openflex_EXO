#!/usr/bin/env python3
import os
from typing import List, Optional, Tuple

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class OpenFlexArmRetargetingNode(Node):
    def __init__(self):
        super().__init__('openflex_arm_retargeting_node')
        self.declare_parameter('config_file', '')
        self.declare_parameter('enable_left_arm_retargeting', True)
        self.declare_parameter('enable_right_arm_retargeting', True)

        self.enable_left = bool(self.get_parameter('enable_left_arm_retargeting').value)
        self.enable_right = bool(self.get_parameter('enable_right_arm_retargeting').value)
        config_file = self.get_parameter('config_file').value
        if not config_file:
            config_file = os.path.join(
                get_package_share_directory('openflex_teleop_exo'),
                'config',
                'retargeting_OpenArmX.yaml',
            )

        with open(config_file, 'r', encoding='utf-8') as f:
            self.cfg = yaml.safe_load(f)

        topics = self.cfg['topics']
        self.exo_joint_topic = topics['exo_joint_topic']
        self.left_topic = topics['target_left_arm_topic']
        self.right_topic = topics['target_right_arm_topic']

        self.left_pub = self.create_publisher(JointState, self.left_topic, 10)
        self.right_pub = self.create_publisher(JointState, self.right_topic, 10)
        self.create_subscription(JointState, self.exo_joint_topic, self._exo_cb, 10)

        self.received = 0
        self.get_logger().info(
            f'OpenFlex arm retargeting started: {self.exo_joint_topic} -> '
            f'{self.left_topic}, {self.right_topic}; config={config_file}'
        )

    def _exo_cb(self, msg: JointState):
        self.received += 1
        positions = list(msg.position)
        if len(positions) == 14:
            positions.extend([0.0, 0.0])
        if len(positions) != 16:
            self.get_logger().warn(f'Ignore EXO joint command with {len(positions)} positions')
            return

        if self.enable_left:
            result = self._retarget_arm(positions, 'left_arm')
            if result is not None:
                joints, gripper = result
                self._publish_arm('left_arm', joints, gripper, msg.header.stamp)

        if self.enable_right:
            result = self._retarget_arm(positions, 'right_arm')
            if result is not None:
                joints, gripper = result
                self._publish_arm('right_arm', joints, gripper, msg.header.stamp)

    def _retarget_arm(self, exo_positions: List[float], arm_side: str) -> Optional[Tuple[List[float], float]]:
        try:
            arm_cfg = self.cfg['exo_joint_mapping'][arm_side]
            exo_arm = [exo_positions[i] for i in arm_cfg['indices']]

            trigger_indices = self.cfg['exo_joint_mapping']['triggers']['indices']
            trigger_index = trigger_indices[0 if arm_side == 'left_arm' else 1]
            gripper_raw = exo_positions[trigger_index] if trigger_index < len(exo_positions) else 0.0
            gripper = float(np.clip(gripper_raw / 0.067, 0.0, 1.0))

            joints = self._apply_transform(exo_arm, arm_side)
            return joints, gripper
        except Exception as exc:
            self.get_logger().error(f'{arm_side} retarget failed: {exc}')
            return None

    def _apply_transform(self, joint_angles: List[float], arm_side: str) -> List[float]:
        params = self.cfg['retargeting_params']
        scale = params['scaling_factors'][arm_side]
        offset = params['offset_angles'][arm_side]
        limits = params['joint_limits'][arm_side]

        retargeted = []
        for i, value in enumerate(joint_angles[:7]):
            transformed = value * scale[i] + offset[i]
            retargeted.append(float(np.clip(transformed, limits['lower'][i], limits['upper'][i])))
        return retargeted

    def _publish_arm(self, arm_side: str, joints: List[float], gripper: float, stamp):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = stamp
        target_cfg = self.cfg['target_joint_mapping'][arm_side]
        msg.name = list(target_cfg['joint_names'])
        msg.position = list(joints)
        msg.velocity = [0.0] * len(joints)
        msg.effort = [0.0] * len(joints)

        if arm_side == 'left_arm':
            msg.name.append('left_gripper_joint')
            msg.position.append(gripper)
            msg.velocity.append(0.0)
            msg.effort.append(0.0)
            self.left_pub.publish(msg)
        else:
            msg.name.append('right_gripper_joint')
            msg.position.append(gripper)
            msg.velocity.append(0.0)
            msg.effort.append(0.0)
            self.right_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpenFlexArmRetargetingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

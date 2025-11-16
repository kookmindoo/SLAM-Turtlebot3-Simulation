"""Multi-topic recorder node for TurtleBot3 sensor streams."""

from __future__ import annotations

import importlib
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from rosbag2_py import (  # type: ignore
    SequentialWriter,
    StorageOptions,
    ConverterOptions,
    TopicMetadata,
)

from std_msgs.msg import String


@dataclass
class TopicConfig:
    name: str
    type: str
    qos: str = 'sensor_data'


class MultiTopicRecorder(Node):
    """Subscribes to multiple sensor topics and writes them to a rosbag."""

    def __init__(self) -> None:
        super().__init__('tb3_multi_topic_recorder')

        self.declare_parameter('bag_path', '/tmp/tb3_dataset')
        self.declare_parameter('storage_id', 'sqlite3')
        self.declare_parameter('serialization_format', 'cdr')
        self.declare_parameter('sync_slop', 0.05)
        self.declare_parameter('topics', [])

        self._bag_path = Path(self.get_parameter('bag_path').get_parameter_value().string_value).expanduser()
        self._storage_id = self.get_parameter('storage_id').get_parameter_value().string_value
        self._serialization_format = self.get_parameter('serialization_format').get_parameter_value().string_value
        self._sync_slop = self.get_parameter('sync_slop').get_parameter_value().double_value
        topic_list = self.get_parameter('topics').get_parameter_value().string_array_value

        if not topic_list:
            self.get_logger().warn('No topics configured, nothing will be recorded.')

        self._topic_configs = self._parse_topic_parameter(topic_list)
        self._writer = self._create_writer()
        self._latest_stamps: Dict[str, float] = {}
        self._subs = []
        self._state_pub = self.create_publisher(String, '/tb3_data_tools/recorder_state', 10)

        for cfg in self._topic_configs:
            msg_type = self._import_message_type(cfg.type)
            qos_profile = self._resolve_qos(cfg.qos)
            topic_metadata = TopicMetadata(
                name=cfg.name,
                type=cfg.type,
                serialization_format=self._serialization_format,
                offered_qos_profiles=''
            )
            self._writer.create_topic(topic_metadata)
            sub = self.create_subscription(
                msg_type,
                cfg.name,
                lambda msg, topic=cfg.name: self._on_msg(topic, msg),
                qos_profile,
            )
            self._subs.append(sub)
            self.get_logger().info(f'Recording {cfg.name} ({cfg.type}) with {cfg.qos} QoS')

        self._bag_path.mkdir(parents=True, exist_ok=True)

    def _create_writer(self) -> SequentialWriter:
        writer = SequentialWriter()
        storage_options = StorageOptions(uri=str(self._bag_path), storage_id=self._storage_id)
        converter_options = ConverterOptions(
            input_serialization_format=self._serialization_format,
            output_serialization_format=self._serialization_format,
        )
        writer.open(storage_options, converter_options)
        return writer

    def _parse_topic_parameter(self, topic_list: List[str]) -> List[TopicConfig]:
        configs: List[TopicConfig] = []
        for entry in topic_list:
            parts = entry.split('|')
            if len(parts) < 2:
                self.get_logger().warn(f'Invalid topic entry: {entry}')
                continue
            name, type_name = parts[0], parts[1]
            qos = parts[2] if len(parts) > 2 else 'sensor_data'
            configs.append(TopicConfig(name=name, type=type_name, qos=qos))
        return configs

    def _import_message_type(self, type_string: str):
        module_name, class_name = type_string.split('/msg/')
        module = importlib.import_module(f'{module_name}.msg')
        return getattr(module, class_name)

    def _resolve_qos(self, qos_name: str) -> QoSProfile:
        qos_name = qos_name.lower()
        if qos_name == 'sensor_data':
            return QoSProfile(  # Sensor data QoS profile
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )
        if qos_name == 'latched':
            return QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1,
            )
        return QoSProfile(depth=10)

    def _on_msg(self, topic_name, msg):
        stamp_msg = self._extract_header_stamp(msg)
        serialized = serialize_message(msg)
        time_ns = Time.from_msg(stamp_msg).nanoseconds
        self._writer.write(topic_name, serialized, time_ns)
        if time_ns:
            self._latest_stamps[topic_name] = time_ns * 1e-9
            self._check_sync()

    def _extract_header_stamp(self, msg):
        header = getattr(msg, 'header', None)
        if header is None:
            return self.get_clock().now().to_msg()
        return header.stamp

    def _check_sync(self):
        if len(self._latest_stamps) < len(self._topic_configs):
            return
        stamps = list(self._latest_stamps.values())
        if max(stamps) - min(stamps) <= self._sync_slop:
            summary = ', '.join(f'{name}:{self._latest_stamps[name]:.3f}' for name in self._latest_stamps)
            msg = String()
            msg.data = f'Synchronized sample captured: {summary}'
            self._state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MultiTopicRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

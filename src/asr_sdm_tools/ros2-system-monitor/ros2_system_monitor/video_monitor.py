#!/usr/bin/env python3

import json
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


IMAGE_TYPE = "sensor_msgs/msg/Image"
COMPRESSED_IMAGE_TYPE = "sensor_msgs/msg/CompressedImage"
SUPPORTED_IMAGE_TYPES = (IMAGE_TYPE, COMPRESSED_IMAGE_TYPE)


@dataclass
class VideoSlotState:
    topic: str = ""
    topic_type: str = ""
    status: str = "No topic selected"
    subscription: Optional[object] = None
    frame_count: int = 0
    generation: int = 0
    last_frame_monotonic: float = 0.0


class VideoMonitor(Node):
    """Relay selected /perception* image topics for the QML monitor UI.

    The node only discovers available video topics by default. It does not
    subscribe to, decode, or forward every /perception* topic. A relay
    subscription is created only after the UI publishes a slot selection on
    /system_monitor/video/select, and it is destroyed as soon as the slot is
    cleared or switched to another topic.
    """

    def __init__(self):
        super().__init__("video_monitor")
        self._lock = threading.RLock()

        self._input_topic_prefix = self._normalize_prefix(
            self.declare_parameter("input_topic_prefix", "/perception")
            .get_parameter_value()
            .string_value
        )
        self._relay_namespace = self._normalize_namespace(
            self.declare_parameter("relay_topic_namespace", "/system_monitor/video")
            .get_parameter_value()
            .string_value
        )
        self._max_slots = max(
            1,
            min(
                2,
                self.declare_parameter("max_slots", 2)
                .get_parameter_value()
                .integer_value,
            ),
        )
        self._discovery_period = max(
            0.2,
            self.declare_parameter("discovery_period", 1.0)
            .get_parameter_value()
            .double_value,
        )
        self._status_period = max(
            0.2,
            self.declare_parameter("status_period", 1.0)
            .get_parameter_value()
            .double_value,
        )
        self._no_frame_warn_sec = max(
            1.0,
            self.declare_parameter("no_frame_warn_sec", 3.0)
            .get_parameter_value()
            .double_value,
        )

        self._latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._sensor_qos = qos_profile_sensor_data

        self._slots: List[VideoSlotState] = [VideoSlotState() for _ in range(self._max_slots)]
        self._available_topic_types: Dict[str, str] = {}
        self._last_available_topics_payload = ""

        self._available_topics_pub = self.create_publisher(
            String,
            f"{self._relay_namespace}/available_topics",
            self._latched_qos,
        )
        self._select_sub = self.create_subscription(
            String,
            f"{self._relay_namespace}/select",
            self._select_callback,
            self._latched_qos,
        )

        self._status_pubs = [
            self.create_publisher(
                String,
                f"{self._relay_namespace}/slot{slot_index}/status",
                self._latched_qos,
            )
            for slot_index in range(self._max_slots)
        ]
        self._image_pubs = [
            self.create_publisher(
                Image,
                f"{self._relay_namespace}/slot{slot_index}/image",
                self._sensor_qos,
            )
            for slot_index in range(self._max_slots)
        ]
        self._compressed_pubs = [
            self.create_publisher(
                CompressedImage,
                f"{self._relay_namespace}/slot{slot_index}/compressed",
                self._sensor_qos,
            )
            for slot_index in range(self._max_slots)
        ]

        self._discovery_timer = self.create_timer(
            self._discovery_period,
            self._discover_and_publish_available_topics,
        )
        self._status_timer = self.create_timer(
            self._status_period,
            self._publish_all_slot_status,
        )

        self._discover_and_publish_available_topics(force=True)
        self._publish_all_slot_status()

        self.get_logger().info(
            "video_monitor ready: discovery prefix='%s', relay namespace='%s', slots=%d"
            % (self._input_topic_prefix, self._relay_namespace, self._max_slots)
        )

    @staticmethod
    def _normalize_prefix(prefix: str) -> str:
        prefix = (prefix or "/perception").strip()
        if not prefix.startswith("/"):
            prefix = "/" + prefix
        return prefix.rstrip("/") or "/perception"

    @staticmethod
    def _normalize_namespace(namespace: str) -> str:
        namespace = (namespace or "/system_monitor/video").strip()
        if not namespace.startswith("/"):
            namespace = "/" + namespace
        return namespace.rstrip("/") or "/system_monitor/video"

    def _discover_video_topics(self) -> List[Tuple[str, str]]:
        discovered: List[Tuple[str, str]] = []
        for topic_name, topic_types in self.get_topic_names_and_types():
            if not topic_name.startswith(self._input_topic_prefix):
                continue

            # Prefer raw Image if a topic somehow appears with multiple types.
            selected_type = ""
            if IMAGE_TYPE in topic_types:
                selected_type = IMAGE_TYPE
            elif COMPRESSED_IMAGE_TYPE in topic_types:
                selected_type = COMPRESSED_IMAGE_TYPE

            if selected_type:
                discovered.append((topic_name, selected_type))

        discovered.sort(key=lambda item: item[0])
        return discovered

    def _discover_and_publish_available_topics(self, force: bool = False):
        try:
            discovered = self._discover_video_topics()
        except Exception as exc:  # graph queries should not kill the monitor
            self.get_logger().warn(f"Failed to discover video topics: {exc}")
            return

        payload = json.dumps(
            {
                "prefix": self._input_topic_prefix,
                "topics": [
                    {"name": name, "type": topic_type}
                    for name, topic_type in discovered
                ],
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )

        with self._lock:
            self._available_topic_types = {name: topic_type for name, topic_type in discovered}
            changed = payload != self._last_available_topics_payload
            if changed:
                self._last_available_topics_payload = payload

        if force or changed:
            msg = String()
            msg.data = payload
            self._available_topics_pub.publish(msg)

    def _lookup_topic_type(self, topic_name: str) -> str:
        with self._lock:
            topic_type = self._available_topic_types.get(topic_name, "")
        if topic_type:
            return topic_type

        for discovered_name, discovered_type in self._discover_video_topics():
            if discovered_name == topic_name:
                return discovered_type
        return ""

    def _select_callback(self, msg: String):
        try:
            payload = json.loads(msg.data) if msg.data else {}
            slot_index = int(payload.get("slot", -1))
            topic_name = str(payload.get("topic", "")).strip()
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f"Ignoring invalid video selection payload: {exc}")
            return

        self._set_slot_topic(slot_index, topic_name)

    def _set_slot_topic(self, slot_index: int, topic_name: str):
        if slot_index < 0 or slot_index >= self._max_slots:
            self.get_logger().warn(f"Ignoring video selection for invalid slot {slot_index}")
            return

        stopped_duplicate_slot: Optional[int] = None

        with self._lock:
            slot = self._slots[slot_index]
            if slot.topic == topic_name:
                self._publish_slot_status_locked(slot_index)
                return

        if not topic_name:
            with self._lock:
                self._stop_slot_locked(slot_index, "No topic selected")
                self._publish_slot_status_locked(slot_index)
            return

        if not topic_name.startswith(self._input_topic_prefix):
            with self._lock:
                self._stop_slot_locked(
                    slot_index,
                    f"Rejected topic outside {self._input_topic_prefix}: {topic_name}",
                )
                self._publish_slot_status_locked(slot_index)
            return

        topic_type = self._lookup_topic_type(topic_name)
        if topic_type not in SUPPORTED_IMAGE_TYPES:
            with self._lock:
                self._stop_slot_locked(
                    slot_index,
                    f"Rejected unsupported or unavailable video topic: {topic_name}",
                )
                self._publish_slot_status_locked(slot_index)
            return

        with self._lock:
            for other_index, other_slot in enumerate(self._slots):
                if other_index != slot_index and other_slot.topic == topic_name:
                    self._stop_slot_locked(other_index, "No topic selected")
                    stopped_duplicate_slot = other_index
                    break

            self._stop_slot_locked(slot_index, "Switching video topic")
            slot = self._slots[slot_index]
            slot.topic = topic_name
            slot.topic_type = topic_type
            slot.status = f"Subscribed, waiting for frames: {topic_name}"
            slot.frame_count = 0
            slot.last_frame_monotonic = 0.0
            slot.generation += 1
            generation = slot.generation

            if topic_type == IMAGE_TYPE:
                slot.subscription = self.create_subscription(
                    Image,
                    topic_name,
                    lambda image_msg, index=slot_index, gen=generation: self._image_callback(index, gen, image_msg),
                    self._sensor_qos,
                )
            else:
                slot.subscription = self.create_subscription(
                    CompressedImage,
                    topic_name,
                    lambda image_msg, index=slot_index, gen=generation: self._compressed_image_callback(index, gen, image_msg),
                    self._sensor_qos,
                )

            self._publish_slot_status_locked(slot_index)
            if stopped_duplicate_slot is not None:
                self._publish_slot_status_locked(stopped_duplicate_slot)

        self.get_logger().info(
            f"slot{slot_index} forwarding {topic_type} from {topic_name}"
        )

    def _stop_slot_locked(self, slot_index: int, status: str):
        slot = self._slots[slot_index]
        if slot.subscription is not None:
            try:
                self.destroy_subscription(slot.subscription)
            except Exception as exc:
                self.get_logger().warn(f"Failed to destroy video subscription for slot{slot_index}: {exc}")

        slot.subscription = None
        slot.topic = ""
        slot.topic_type = ""
        slot.status = status
        slot.frame_count = 0
        slot.last_frame_monotonic = 0.0
        slot.generation += 1

    def _image_callback(self, slot_index: int, generation: int, msg: Image):
        should_publish = False
        with self._lock:
            if slot_index >= self._max_slots:
                return
            slot = self._slots[slot_index]
            if slot.generation != generation or slot.topic_type != IMAGE_TYPE:
                return
            slot.frame_count += 1
            slot.last_frame_monotonic = time.monotonic()
            slot.status = f"Forwarding raw image frames: {slot.frame_count}"
            should_publish = True

        if should_publish:
            self._image_pubs[slot_index].publish(msg)

    def _compressed_image_callback(self, slot_index: int, generation: int, msg: CompressedImage):
        should_publish = False
        with self._lock:
            if slot_index >= self._max_slots:
                return
            slot = self._slots[slot_index]
            if slot.generation != generation or slot.topic_type != COMPRESSED_IMAGE_TYPE:
                return
            slot.frame_count += 1
            slot.last_frame_monotonic = time.monotonic()
            slot.status = f"Forwarding compressed image frames: {slot.frame_count}"
            should_publish = True

        if should_publish:
            self._compressed_pubs[slot_index].publish(msg)

    def _publish_all_slot_status(self):
        with self._lock:
            now = time.monotonic()
            for slot_index, slot in enumerate(self._slots):
                if slot.topic and slot.last_frame_monotonic > 0.0:
                    age = now - slot.last_frame_monotonic
                    if age > self._no_frame_warn_sec:
                        slot.status = "No video frame for %.1f s" % age
                self._publish_slot_status_locked(slot_index)

    def _publish_slot_status_locked(self, slot_index: int):
        slot = self._slots[slot_index]
        payload = {
            "slot": slot_index,
            "selected_topic": slot.topic,
            "topic_type": slot.topic_type,
            "status": slot.status,
            "frame_count": slot.frame_count,
        }
        if slot.last_frame_monotonic > 0.0:
            payload["seconds_since_last_frame"] = round(
                time.monotonic() - slot.last_frame_monotonic,
                3,
            )

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self._status_pubs[slot_index].publish(msg)

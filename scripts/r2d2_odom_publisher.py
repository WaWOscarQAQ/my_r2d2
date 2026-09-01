#!/usr/bin/env python3
"""Publish a tiny loopback odom/TF stream for local Nav2 full-stack fuzzing."""

import math
import os

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, TwistStamped
import rclpy
from nav_msgs.msg import Odometry
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class R2D2OdomPublisher(Node):
    def __init__(self) -> None:
        super().__init__("r2d2_odom_publisher")
        self.x = float(os.environ.get("R2D2_START_X", "-1.5"))
        self.y = float(os.environ.get("R2D2_START_Y", "0.0"))
        self.yaw = float(os.environ.get("R2D2_START_YAW", "0.0"))
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.last_update = self.get_clock().now()
        self.last_cmd = self.last_update
        self.publisher = self.create_publisher(Odometry, "odom", QoSProfile(depth=10))
        self.cmd_vel_topics = self.parse_cmd_vel_topics()
        for topic in self.cmd_vel_topics:
            self.create_subscription(
                TwistStamped,
                topic,
                self.cmd_vel_callback,
                QoSProfile(depth=10),
            )
        self.initial_pose_topic = os.environ.get("R2D2_INITIAL_POSE_TOPIC", "initialpose")
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            self.initial_pose_callback,
            QoSProfile(depth=10),
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transforms()
        self.timer = self.create_timer(0.02, self.publish_odom)
        self.get_logger().info(
            "Listening for TwistStamped commands on: " + ", ".join(self.cmd_vel_topics)
        )
        self.get_logger().info(
            "Listening for initial pose resets on: " + self.initial_pose_topic
        )

    @staticmethod
    def parse_cmd_vel_topics() -> list[str]:
        configured = os.environ.get("R2D2_CMD_VEL_TOPICS", "cmd_vel,cmd_vel_nav")
        topics: list[str] = []
        for item in configured.split(","):
            topic = item.strip()
            if topic and topic not in topics:
                topics.append(topic)
        return topics or ["cmd_vel"]

    def publish_static_transforms(self) -> None:
        stamp = self.get_clock().now().to_msg()
        transforms = [
            self.make_transform("map", "odom", stamp),
            self.make_transform("base_footprint", "base_link", stamp),
            self.make_transform("base_link", "base_scan", stamp),
            self.make_transform("base_link", "laser_frame", stamp),
        ]
        self.static_tf_broadcaster.sendTransform(transforms)
        self.get_logger().info(
            "Published static TF chain: map->odom, base_footprint->base_link, "
            "base_link->base_scan, base_link->laser_frame"
        )

    @staticmethod
    def make_transform(
        parent: str,
        child: str,
        stamp,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        yaw: float = 0.0,
    ) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        transform.transform.rotation.z = math.sin(yaw * 0.5)
        transform.transform.rotation.w = math.cos(yaw * 0.5)
        return transform

    def cmd_vel_callback(self, msg: TwistStamped) -> None:
        self.linear_x = float(msg.twist.linear.x)
        self.linear_y = float(msg.twist.linear.y)
        self.angular_z = float(msg.twist.angular.z)
        self.last_cmd = self.get_clock().now()

    def initial_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        orientation = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        now = self.get_clock().now()
        self.last_update = now
        self.last_cmd = now

    def publish_odom(self) -> None:
        now = self.get_clock().now()
        dt = max(0.0, min((now - self.last_update).nanoseconds / 1_000_000_000.0, 0.2))
        self.last_update = now
        if (now - self.last_cmd).nanoseconds > 500_000_000:
            self.linear_x = 0.0
            self.linear_y = 0.0
            self.angular_z = 0.0

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        self.x += (self.linear_x * cos_yaw - self.linear_y * sin_yaw) * dt
        self.y += (self.linear_x * sin_yaw + self.linear_y * cos_yaw) * dt
        self.yaw = math.atan2(
            math.sin(self.yaw + self.angular_z * dt),
            math.cos(self.yaw + self.angular_z * dt),
        )
        quat_z = math.sin(self.yaw * 0.5)
        quat_w = math.cos(self.yaw * 0.5)

        stamp = now.to_msg()
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_footprint"
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = quat_z
        msg.pose.pose.orientation.w = quat_w
        msg.twist.twist.linear.x = self.linear_x
        msg.twist.twist.linear.y = self.linear_y
        msg.twist.twist.angular.z = self.angular_z
        self.publisher.publish(msg)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = quat_z
        transform.transform.rotation.w = quat_w
        self.tf_broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = R2D2OdomPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

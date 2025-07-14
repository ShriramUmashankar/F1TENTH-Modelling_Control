#!/usr/bin/env python3

import time
import threading
from collections import deque
from math import atan2, pi
import argparse

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion, Vector3
from std_msgs.msg import Header
from nav_msgs.msg import Odometry

import natnet


def quaternion_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return atan2(siny_cosp, cosy_cosp)


class MocapOdometryNode(Node):
    def __init__(self, server_ip, target_id):
        super().__init__('mocap_odometry_node')

        self._target_id = target_id

        # Publishers
        self._odom_pub = self.create_publisher(Odometry, 'mocap/odom', 10)
        self._odom_raw_pub = self.create_publisher(Odometry, 'mocap/odom_raw', 10)

        # Filter windows
        self._dt_window = deque([1 / 120.0] * 10, maxlen=10)
        self._vel_window = deque([(0.0, 0.0, 0.0)] * 10, maxlen=10)
        self._angvel_window = deque([0.0] * 10, maxlen=10)

        self._start_time = None
        self._prev_time = None
        self._prev_pos = None
        self._prev_yaw = None

        # Connect NatNet client
        self._client = natnet.Client.connect(server_ip)
        if self._client is None:
            raise RuntimeError("Failed to connect to NatNet server")

        self._client.set_callback(self.callback)

        # Spin NatNet in background thread
        self._natnet_thread = threading.Thread(target=self._client.spin, daemon=True)
        self._natnet_thread.start()

        # 120 Hz publishing loop
        self._timer = self.create_timer(1.0 / 120.0, self.publish_odometry)

        self._latest_odom = None

    def callback(self, rigid_bodies, markers, timing):
        for b in rigid_bodies:
            if b.id_ != self._target_id:
                continue    

            now = time.time()
            pos = b.position
            qx, qy, qz, qw = b.orientation
            yaw = quaternion_to_yaw(qx, qy, qz, qw)

            if self._start_time is None:
                self._start_time = now
                self._prev_time = now
                self._prev_pos = pos
                self._prev_yaw = yaw
                return

            dt = now - self._prev_time
            if dt <= 0:
                return

            # Compute raw velocities
            dx = pos[0] - self._prev_pos[0]
            dy = pos[1] - self._prev_pos[1]
            dz = pos[2] - self._prev_pos[2]
            vx_raw, vy_raw, vz_raw = dx / dt, dy / dt, dz / dt

            dyaw = (yaw - self._prev_yaw + pi) % (2 * pi) - pi
            yaw_rate_raw = dyaw / dt

            # ROS timestamp
            t_ros = self.get_clock().now().to_msg()

            # Publish raw odometry
            odom_raw = Odometry()
            odom_raw.header = Header()
            odom_raw.header.stamp = t_ros
            odom_raw.header.frame_id = 'mocap'
            odom_raw.child_frame_id = 'rigid_body'

            odom_raw.pose.pose.position = Point(x=pos[0], y=pos[1], z=pos[2])
            odom_raw.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            odom_raw.twist.twist.linear = Vector3(x=vx_raw, y=vy_raw, z=vz_raw)
            odom_raw.twist.twist.angular = Vector3(x=0.0, y=0.0, z=yaw_rate_raw)

            self._odom_raw_pub.publish(odom_raw)

            # Filtered version
            self._dt_window.append(dt)
            avg_dt = sum(self._dt_window) / len(self._dt_window)

            self._vel_window.append((vx_raw, vy_raw, vz_raw))
            avg_vx = sum(v[0] for v in self._vel_window) / len(self._vel_window)
            avg_vy = sum(v[1] for v in self._vel_window) / len(self._vel_window)
            avg_vz = sum(v[2] for v in self._vel_window) / len(self._vel_window)

            self._angvel_window.append(yaw_rate_raw)
            avg_yaw_rate = sum(self._angvel_window) / len(self._angvel_window)

            odom = Odometry()
            odom.header = Header()
            odom.header.stamp = t_ros
            odom.header.frame_id = 'mocap'
            odom.child_frame_id = 'rigid_body'

            odom.pose.pose.position = Point(x=pos[0], y=pos[1], z=pos[2])
            odom.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            odom.twist.twist.linear = Vector3(x=avg_vx, y=avg_vy, z=avg_vz)
            odom.twist.twist.angular = Vector3(x=0.0, y=0.0, z=avg_yaw_rate)

            self._latest_odom = odom

            # Update previous state
            self._prev_time = now
            self._prev_pos = pos
            self._prev_yaw = yaw

    def publish_odometry(self):
        if self._latest_odom is not None:
            self._odom_pub.publish(self._latest_odom)


def main():
    parser = argparse.ArgumentParser(description="Mocap Odometry Publisher for ROS 2")
    parser.add_argument('--target-id', type=int, default=47, help='Rigid body ID to track (default: 47)')
    parser.add_argument('--server-ip', type=str, default="130.233.123.181", help='NatNet server IP address')
    args = parser.parse_args()

    rclpy.init()

    try:
        node = MocapOdometryNode(server_ip=args.server_ip, target_id=args.target_id)
        rclpy.spin(node)
    except natnet.DiscoveryError as e:
        print(f"Discovery error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import TransformStamped
import tf2_ros


class ArucoToTfBridge(Node):
    def __init__(self):
        super().__init__('aruco_to_tf_bridge')
        self.declare_parameter('marker_frame_prefix', 'marker_')
        self.prefix = self.get_parameter('marker_frame_prefix').value

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(
            ArucoMarkers, '/aruco_markers',
            self.markers_callback, 10)
        self.get_logger().info('ArucoToTfBridge ready.')

    def markers_callback(self, msg: ArucoMarkers):
        for i, marker_id in enumerate(msg.marker_ids):
            t = TransformStamped()
            t.header           = msg.header  # frame_id = camera_color_optical_frame
            t.child_frame_id   = f'{self.prefix}{marker_id}'  # 'marker_0', 'marker_1'
            t.transform.translation.x = msg.poses[i].position.x
            t.transform.translation.y = msg.poses[i].position.y
            t.transform.translation.z = msg.poses[i].position.z
            t.transform.rotation      = msg.poses[i].orientation
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ArucoToTfBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
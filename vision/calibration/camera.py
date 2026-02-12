#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import pyrealsense2 as rs
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

# --- 설정값 ---
MARKER_SIZE = 0.05  # 마커 크기 (미터)
ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50
TOPIC_NAMESPACE = '/cam0'
FRAME_ID = 'camera_color_optical_frame'

class ArucoRealsenseNode(Node):
    def __init__(self):
        super().__init__('aruco_realsense_publisher')
        
        self.publishers_dict = {}
        
        # RealSense 설정
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 6)
        self.profile = self.pipeline.start(config)
        
        # 카메라 파라미터 획득
        self.camera_matrix, self.dist_coeffs = self.get_intrinsics()
        
        # ArUco 초기화
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # 마커 3D 좌표
        self.marker_points = np.array([
            [-MARKER_SIZE / 2, MARKER_SIZE / 2, 0],
            [MARKER_SIZE / 2, MARKER_SIZE / 2, 0],
            [MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
            [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0]
        ], dtype=np.float32)

        self.timer = self.create_timer(0.166, self.timer_callback)
        self.get_logger().info(f'Node Started with GUI. Window: "Aruco Detection"')

        self.current_frame = None  # 화면에 표시할 프레임 저장용

    def get_intrinsics(self):
        stream = self.profile.get_stream(rs.stream.color)
        intrinsics = stream.as_video_stream_profile().get_intrinsics()
        camera_matrix = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
        ], dtype=np.float32)
        dist_coeffs = np.array(intrinsics.coeffs, dtype=np.float32)
        return camera_matrix, dist_coeffs

    def timer_callback(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                return

            frame = np.asanyarray(color_frame.get_data())
            # GUI용 복사본 생성 (원본 유지)
            display_frame = frame.copy()
            
            # 마커 탐지
            corners, ids, rejected = self.detector.detectMarkers(frame)

            if ids is not None and len(ids) > 0:
                # [추가] 탐지된 마커 테두리 그리기
                cv2.aruco.drawDetectedMarkers(display_frame, corners, ids)

                for i, marker_id in enumerate(ids.flatten()):
                    current_corners = corners[i]
                    
                    success, rvec, tvec = cv2.solvePnP(
                        self.marker_points, current_corners, 
                        self.camera_matrix, self.dist_coeffs
                    )

                    if success:
                        # [추가] 마커 중심에 축(Axis) 그리기 (X:빨강, Y:초록, Z:파랑)
                        cv2.drawFrameAxes(display_frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)

                        # ROS 2 토픽 발행
                        if marker_id not in self.publishers_dict:
                            topic_name = f'{TOPIC_NAMESPACE}/marker_{marker_id}'
                            self.publishers_dict[marker_id] = self.create_publisher(PoseStamped, topic_name, 10)
                            self.get_logger().info(f'New topic: {topic_name}')

                        self.publish_pose(self.publishers_dict[marker_id], rvec, tvec)

            # [추가] GUI 창에 프레임 표시
            self.current_frame = display_frame
        

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def publish_pose(self, publisher, rvec, tvec):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID
        msg.pose.position.x = tvec[0][0]
        msg.pose.position.y = tvec[1][0]
        msg.pose.position.z = tvec[2][0]

        rotation_matrix, _ = cv2.Rodrigues(rvec)
        r = R.from_matrix(rotation_matrix)
        quat = r.as_quat()
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        publisher.publish(msg)

    def __del__(self):
        try:
            self.pipeline.stop()
            cv2.destroyAllWindows()
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = ArucoRealsenseNode()
    
    try:
        # rclpy.spin(node) 대신 직접 루프를 제어
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01) # ROS 이벤트 처리
            
            # 메인 스레드에서 화면 그리기
            if node.current_frame is not None:
                cv2.imshow("Aruco Detection", node.current_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
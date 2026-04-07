import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import pyrealsense2 as rs
import numpy as np
import cv2

class VisionPublisher(Node):
    def __init__(self):
        super().__init__('vision_publisher')
        
        # /cam0 토픽 발행자 생성
        self.publisher_ = self.create_publisher(Point, '/cam0', 10)
        
        # 퍼블리시 주기 설정 (10Hz)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # RealSense 파이프라인 초기화
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        # ArUco 설정 (기존 프로젝트 기준 DICT_4X4_50 사용)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # OpenCV 버전에 따른 Detector 초기화 (호환성 유지)
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            self.detector = None

        try:
            self.profile = self.pipeline.start(self.config)
            self.get_logger().info('RealSense 카메라가 성공적으로 연결되었습니다.')
            
            # 카메라 정렬 설정
            self.align = rs.align(rs.stream.color)
            
        except Exception as e:
            self.get_logger().error(f'카메라 초기화 실패: {e}')
            self.pipeline = None

    def timer_callback(self):
        if self.pipeline is None:
            return

        try:
            # 프레임 획득 및 정렬
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                return
                
            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
            color_image = np.asanyarray(color_frame.get_data())
            
            # ArUco 마커 탐지
            if self.detector:
                corners, ids, _ = self.detector.detectMarkers(color_image)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(color_image, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None:
                # 탐지된 마커 시각화
                cv2.aruco.drawDetectedMarkers(color_image, corners, ids)
                
                # 첫 번째 탐지된 마커의 중심점 계산
                for i in range(len(ids)):
                    c = corners[i][0]
                    target_u = int(np.mean(c[:, 0]))
                    target_v = int(np.mean(c[:, 1]))
                    
                    # 마커 중심점 깊이값 획득 (단위: 미터)
                    dist_to_target = depth_frame.get_distance(target_u, target_v)
                    
                    if dist_to_target > 0:
                        # 3D 공간 좌표 변환 (카메라 좌표계 기준 x, y, z)
                        point_3d = rs.rs2_deproject_pixel_to_point(depth_intrin, [target_u, target_v], dist_to_target)
                        
                        # ROS 2 메시지 발행
                        msg = Point()
                        msg.x = float(point_3d[0])
                        msg.y = float(point_3d[1])
                        msg.z = float(point_3d[2])
                        self.publisher_.publish(msg)
                        
                        self.get_logger().info(f'Marker ID {ids[i][0]} - X:{msg.x:.4f}, Y:{msg.y:.4f}, Z:{msg.z:.4f}')
                        
                        # 시각화: 탐지된 중심점에 점 표시
                        cv2.circle(color_image, (target_u, target_v), 5, (0, 255, 0), -1)
                        break # 예시로 첫 번째 마커 정보만 발행

            cv2.imshow('RealSense ArUco Tracking', color_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'프레임 처리 에러: {e}')

def main(args=None):
    rclpy.init(args=args)
    vision_publisher = VisionPublisher()
    try:
        rclpy.spin(vision_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if vision_publisher.pipeline:
            vision_publisher.pipeline.stop()
        cv2.destroyAllWindows()
        vision_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
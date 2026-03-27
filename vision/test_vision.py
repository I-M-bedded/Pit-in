import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import pyrealsense2 as rs
import numpy as np
import cv2

class VisionPublisher(Node):
    def __init__(self):
        super().__init__('vision_publisher')
        
        # /cam0 토픽 발행자 생성 (interface.py의 lcam_subscription과 일치)
        self.publisher_ = self.create_publisher(Point, '/cam0', 10)
        
        # 퍼블리시 주기 설정 (예: 10Hz)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # RealSense 파이프라인 초기화
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # 해상도 및 프레임 속도 설정 (필요에 따라 수정)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        try:
            self.profile = self.pipeline.start(self.config)
            self.depth_sensor = self.profile.get_device().first_depth_sensor()
            self.depth_scale = self.depth_sensor.get_depth_scale()
            self.get_logger().info('RealSense 카메리가 성공적으로 연결되었습니다.')
            
            # 카메라 내부 파라미터(Intrinsics) 가져오기
            self.align_to = rs.stream.color
            self.align = rs.align(self.align_to)
            
        except Exception as e:
            self.get_logger().error(f'카메라 초기화 실패: {e}')
            self.pipeline = None

    def timer_callback(self):
        if self.pipeline is None:
            return

        try:
            # 프레임 대기
            frames = self.pipeline.wait_for_frames()
            
            # 컬러 프레임과 깊이 프레임 정렬
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                return
                
            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

            color_image = np.asanyarray(color_frame.get_data())
            # depth_image = np.asanyarray(depth_frame.get_data())

            # ==========================================================
            # [Vision Detection Logic]
            # 여기에 YOLO 또는 영상처리 로직을 추가하여 목표(Hole)의 픽셀 좌표(u, v)를 찾습니다.
            # (현재 예시: 임시로 이미지 중심점 근처에 가상의 타겟이 있다고 가정)
            
            # 임시 픽셀 타겟 (실제로는 모델 추론 결과값으로 대체하세요)
            target_u = color_image.shape[1] // 2
            target_v = color_image.shape[0] // 2
            
            # 시각화 (확인용)
            cv2.circle(color_image, (target_u, target_v), 5, (0, 255, 0), -1)
            cv2.imshow('RealSense Left Cam Tracking', color_image)
            cv2.waitKey(1)
            # ==========================================================

            # 해당 픽셀의 깊이값 획득 (단위: 미터)
            dist_to_target = depth_frame.get_distance(target_u, target_v)
            
            # 깊이값이 유효한 경우 3D 공간 좌표계로 변환 (Deprojection)
            if dist_to_target > 0:
                # 결과 좌표 point_3d는 카메라 좌표계 기준 [x, y, z] (미터 단위)
                # Realsense 좌표계: x(우측), y(아래), z(전방)
                point_3d = rs.rs2_deproject_pixel_to_point(depth_intrin, [target_u, target_v], dist_to_target)
                
                # Point 메시지 생성 및 발행
                msg = Point()
                msg.x = float(point_3d[0])
                msg.y = float(point_3d[1])
                msg.z = float(point_3d[2])
                
                self.publisher_.publish(msg)
                
                # 터미널 로깅
                self.get_logger().info(f'Published Target to /cam0 (m): x={msg.x:.4f}, y={msg.y:.4f}, z={msg.z:.4f}')
            
        except Exception as e:
            self.get_logger().error(f'프레임 처리 중 에러 발생: {e}')

def main(args=None):
    rclpy.init(args=args)
    vision_publisher = VisionPublisher()
    
    try:
        rclpy.spin(vision_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 카메라 파이프라인 안전하게 닫기
        if vision_publisher.pipeline:
            vision_publisher.pipeline.stop()
        cv2.destroyAllWindows()
        vision_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

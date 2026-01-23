from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from datetime import datetime

class HandEyeDataCollector(Node):
    def __init__(self, target_marker_id=0):
        super().__init__('hand_eye_collector')
        
        # [설정] 수집할 마커 ID 지정 (예: 0번 마커)
        self.target_topic = f'camera/pose'
        
        # 1. 카메라 토픽 구독
        # 멀티캠 코드에서 같은 토픽으로 보내되 frame_id로 L/R을 구분하므로 하나만 구독하면 됩니다.
        self.subscription = self.create_subscription(
            PoseStamped,
            self.target_topic,
            self.listener_callback,
            10
        )
        
        # 2. 데이터 저장 변수 초기화 (Left, Right 각각 관리)
        self.latest_cam_L = None
        self.latest_cam_R = None
        
            
        self.get_logger().info(f"Ready to record Dual Camera Data. Topic: {self.target_topic}")
        self.count = 0

    def listener_callback(self, msg_cam):
        """
        토픽이 들어올 때마다 실행됩니다.
        frame_id를 확인하여 왼쪽/오른쪽 데이터를 갱신합니다.
        """
        # 현재 메시지가 어느 카메라에서 왔는지 확인 (cam_L_link 또는 cam_R_link)
        
        # 데이터 추출
        x = msg_cam.pose.position.x
        y = msg_cam.pose.position.y
        z = msg_cam.pose.position.z
        
        # frame_id에 따라 변수 업데이트
        self.latest_cam_L = [x, y, z]
            # 디버깅용 로그 (필요시 주석 해제)
            #self.get_logger().info(f"Update Left: {z:.3f}m")
            
        self.latest_cam_R = [x-1.4, y, z]
            #self.get_logger().info(f"Update Right: {z:.3f}m")
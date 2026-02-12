import rclpy
from rclpy.node import Node
from pitin_msgs.msg import Agvop, Srvop, Amonitoring, JointState
from geometry_msgs.msg import Point, PoseStamped # PoseStamped 추가

class SrvreqSubscriber(Node):

    def __init__(self, agv, servoState):
        super().__init__('agv_planning')
        ares_period = 1.0
        amonitor_period = 0.2
        ajoint_period = 0.1
        # [추가] 마커 토픽 감시 주기 (1초)
        marker_scan_period = 1.0
        
        self.agv = agv
        self.servoState = servoState
        self.server_req = 'server_req_ver1'
        self.agv_res = 'agv_res_ver1'
        self.agv_monitoring = 'agv_monitoring_ver1'
        self.joint_state = 'agv_joint_state'
        self.lcam = 'cam0'
        self.rcam = 'cam1' 

        # [추가] 동적 마커 구독 관리를 위한 딕셔너리
        # key: topic_name, value: subscription object
        self.marker_subscriptions = {}
        # AGV 클래스 내부에 마커 데이터를 저장할 공간이 있다고 가정 (dict 형태 추천)
        if not hasattr(self.agv, 'detected_markers'):
            self.agv.detected_markers = {}

        # 기본 서비스/명령 구독
        self.subscription = self.create_subscription(Srvop, self.server_req, self.listener_callback, 10)
        
        # 기존 고정 카메라 리스너 (Point 타입 유지)
        self.lcam_subscription = self.create_subscription(Point, '/cam0', self.lcam_listener_callback, 10)
        self.rcam_subscription = self.create_subscription(Point, '/cam1', self.rcam_listener_callback, 10)
        
        # 발행자 설정
        self.publisher_ = self.create_publisher(Amonitoring, self.agv_monitoring, 10)
        self.publisher2_ = self.create_publisher(Agvop, self.agv_res, 10)
        self.publisher3_ = self.create_publisher(JointState, self.joint_state, 10)

        # 타이머 설정
        self.timer_monitor = self.create_timer(amonitor_period, self.agvmonitoring_callback)
        self.timer_res = self.create_timer(ares_period, self.agv_res_auto)
        self.timer_joint = self.create_timer(ajoint_period, self.agv_joint_update)
        
        # [추가] 동적 토픽 스캔 타이머
        self.timer_marker_scan = self.create_timer(marker_scan_period, self.scan_for_marker_topics)

    # [추가] 새로운 마커 토픽이 있는지 검사하는 함수
    def scan_for_marker_topics(self):
        topic_names_and_types = self.get_topic_names_and_types()
        
        for topic_name, topic_types in topic_names_and_types:
            # 우리가 찾는 형식 '/cam0/marker_ID' 인지 확인
            if '/cam0/marker_' in topic_name and 'geometry_msgs/msg/PoseStamped' in topic_types:
                # 아직 구독 중이지 않은 새로운 토픽이라면 구독 생성
                if topic_name not in self.marker_subscriptions:
                    self.get_logger().info(f'Found new marker topic: {topic_name}. Subscribing...')
                    
                    # 콜백에서 어떤 토픽인지 알 수 있도록 lambda 사용
                    sub = self.create_subscription(
                        PoseStamped,
                        topic_name,
                        lambda msg, t_name=topic_name: self.marker_callback(msg, t_name),
                        10
                    )
                    self.marker_subscriptions[topic_name] = sub

    # [추가] 동적 마커용 통합 콜백
    def marker_callback(self, msg, topic_name):
        # 토픽명에서 ID 추출 (예: /cam0/marker_5 -> 5)
        try:
            marker_id = int(topic_name.split('_')[-1])
            # agv 객체의 데이터 구조에 맞게 저장
            self.agv.detected_markers[marker_id] = {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z,
                'qx': msg.pose.orientation.x,
                'qy': msg.pose.orientation.y,
                'qz': msg.pose.orientation.z,
                'qw': msg.pose.orientation.w,
                'stamp': msg.header.stamp
            }
            # 디버깅 로그 (필요 시 해제)
            # self.get_logger().info(f'Marker {marker_id} updated')
        except Exception as e:
            self.get_logger().error(f'Error in marker_callback: {e}')

    def lcam_listener_callback(self, msg):
        self.agv.lcam_hole_pos[0] = msg.x
        self.agv.lcam_hole_pos[1] = msg.y
        self.agv.lcam_hole_pos[2] = msg.z
    
    def rcam_listener_callback(self, msg):
        self.agv.rcam_hole_pos[0] = msg.x
        self.agv.rcam_hole_pos[1] = msg.y
        self.agv.rcam_hole_pos[2] = msg.z

    # --- 기존의 다른 메서드들 (listener_callback, agvmonitoring_callback 등)은 동일하게 유지 ---
    def listener_callback(self, msg):
        agv=self.agv
        pmsg=Agvop()
        agv.reqid_ros2[2][0]=msg.reqid_s
        agv.reqid_ros2[2][1]=msg.seqid_s
        # ... (이하 생략, 기존 코드와 동일)
        self.publisher2_.publish(pmsg)

    def agvmonitoring_callback(self):
        msg=Amonitoring()
        # ... (기존 필드 설정)
        msg.stamp=self.get_clock().now().to_msg()
        self.publisher_.publish(msg)

    def agv_res_auto(self):
        if self.agv.reqid_ros2[0][0]!=0:
            msg=Agvop()
            # ... (기존 로직)
            self.publisher2_.publish(msg)

    def agv_joint_update(self):
        msg=JointState()
        # ... (기존 로직)
        self.publisher3_.publish(msg)

def agv_planning_main(agv, servostate):
    rclpy.init(args=None)
    srv_sub = SrvreqSubscriber(agv, servostate)
    rclpy.spin(srv_sub)
    srv_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # 상대 경로 임포트 문제 방지를 위한 가이드
    # 실제 환경에서는 실행 위치에 맞춰 조정 필요
    try:
        from ..leadshine.agv_con import AGV
    except ImportError:
        class AGV: pass # 더미 클래스 (테스트용)
    agv_planning_main(agv=AGV(), servostate=None)
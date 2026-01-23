import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
import time
import os
from datetime import datetime

class HandEyeDataCollector(Node):
    def __init__(self, robot_instance): # robot_instance를 받도록 복구
        super().__init__('hand_eye_collector')
        self.robot = robot_instance
        
        # 1. 카메라 토픽 구독 (데이터 수신용)
        self.subscription_set1 = self.create_subscription(
            PoseStamped,
            '/aruco/marker_0/pose',
            self.listener_callback_set1,
            10
        )
        self.subscription_set2 = self.create_subscription(
            PoseStamped,
            '/aruco/marker_2/pose',
            self.listener_callback_set2,
            10
        )
            
        self.get_logger().info(f"Ready to record. File: {self.filename}")
        self.count = 0
        self.latest_cam_data_set1 = None
        self.latest_cam_data_set2 = None
        
    def listener_callback_set1(self, msg_cam):
        cx_L = msg_cam.pose.position.x
        cy_L = msg_cam.pose.position.y
        cz_L = msg_cam.pose.position.z
        cx_R = msg_cam.pose.position.x
        cy_R = msg_cam.pose.position.y
        cz_R = msg_cam.pose.position.z

        # 최신 데이터 갱신
        self.latest_cam_data_set1 = [cx_L, cy_L, cz_L,cx_R,cy_R,cz_R]

    def listener_callback_set2(self, msg_cam):
        cx_L = msg_cam.pose.position.x
        cy_L = msg_cam.pose.position.y
        cz_L = msg_cam.pose.position.z
        cx_R = cx_L-144
        cy_R = cy_L
        cz_R = cz_L

        # 최신 데이터 갱신
        self.latest_cam_data_set2 = [cx_L, cy_L, cz_L,cx_R,cy_R,cz_R]


    def save_snapshot(self,angle):
        """
        ★ 우리가 호출할 때만 실행되는 저장 함수 ★
        """
        # 1. 카메라 데이터가 아직 한 번도 안 왔으면 무시
        if self.latest_cam_data_set1 is None:
            print("[Warn] No Camera Data yet!")
            return

        try:
            # 2. 로봇 데이터 가져오기 (호출된 그 순간의 좌표)
            # pos_target_oper.X0 구조에 맞게 가져옴
            robot_pos = self.robot.pto.X1
            zloc=self.robot.pto.zloc
            rx, ry, rz = robot_pos[0], robot_pos[1], 0
            
            # 3. 카메라 데이터 가져오기
            _,_,_,cx, cy, cz = self.latest_cam_data_set

            # 4. 파일에 한 줄 저장
            current_time = time.time()
            with open(self.filename, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    current_time,
                    rx, ry, rz,
                    cx, cy, cz,
                    -angle
                ])
            
            self.count += 1
            print(f">>> [Saved #{self.count}] Rob:({rx:.1f}, {ry:.1f}, {rz:.1f}) | Cam:({cx:.2f}, {cy:.2f}, {cz:.2f})")

        except Exception as e:
            print(f"[Error] Save failed: {e}")
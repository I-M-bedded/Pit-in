try: from ros2 import interface as rif
except: pass
from sky import handeyedata_multi

def agv_planning_main(self):
    rif.rclpy.init(args=None)
    self.ros = rif.SrvreqSubscriber(self.agv,self.servo_states)
    self.cali_node = handeyedata_multi.HandEyeDataCollector(target_marker_id=2)
    self.thereadcheck[0]=1
    
    # 두 노드 동시 실행
    executor = rif.rclpy.executors.MultiThreadedExecutor()
    executor.add_node(self.ros)
    executor.add_node(self.cali_node)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        self.ros.destroy_node()
        self.cali_node.destroy_node()
        rif.rclpy.shutdown()
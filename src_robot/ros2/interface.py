import rclpy
from rclpy.node import Node
from pitin_msgs.msg import Agvop, Srvop, Amonitoring, JointState
from geometry_msgs.msg import Point, PoseStamped


class SrvreqSubscriber(Node):

    def __init__(self, agv, servoState):
        super().__init__('agv_planning')
        ares_period = 1.0
        amonitor_period = 0.2
        ajoint_period = 0.1
        marker_scan_period = 1.0

        self.agv = agv
        self.servoState = servoState
        self.server_req = 'server_req_ver1'
        self.agv_res = 'agv_res_ver1'
        self.agv_monitoring = 'agv_monitoring_ver1'
        self.joint_state = 'agv_joint_state'
        self.lcam = 'cam0'
        self.rcam = 'cam1'

        # 동적 마커 구독 관리
        self.marker_subscriptions = {}
        if not hasattr(self.agv, 'detected_markers'):
            self.agv.detected_markers = {}

        # server_req topic listener
        self.subscription = self.create_subscription(
            Srvop,
            self.server_req,
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # camera listener
        self.lcam_subscription = self.create_subscription(
            Point,
            '/cam0',
            self.lcam_listener_callback,
            10
        )
        self.lcam_subscription

        self.rcam_subscription = self.create_subscription(
            Point,
            '/cam1',
            self.rcam_listener_callback,
            10
        )
        self.rcam_subscription

        # publishers
        self.publisher_ = self.create_publisher(Amonitoring, self.agv_monitoring, 10)
        self.publisher2_ = self.create_publisher(Agvop, self.agv_res, 10)
        self.publisher3_ = self.create_publisher(JointState, self.joint_state, 10)

        # timers
        self.timer_monitor = self.create_timer(amonitor_period, self.agvmonitoring_callback)
        self.timer_res = self.create_timer(ares_period, self.agv_res_auto)
        self.timer_joint = self.create_timer(ajoint_period, self.agv_joint_update)
        self.timer_marker_scan = self.create_timer(marker_scan_period, self.scan_for_marker_topics)

    def scan_for_marker_topics(self):
        topic_names_and_types = self.get_topic_names_and_types()

        for topic_name, topic_types in topic_names_and_types:
            if '/cam0/marker_' in topic_name and 'geometry_msgs/msg/PoseStamped' in topic_types:
                if topic_name not in self.marker_subscriptions:
                    self.get_logger().info(f'Found new marker topic: {topic_name}. Subscribing...')
                    sub = self.create_subscription(
                        PoseStamped,
                        topic_name,
                        lambda msg, t_name=topic_name: self.marker_callback(msg, t_name),
                        10
                    )
                    self.marker_subscriptions[topic_name] = sub

    def marker_callback(self, msg, topic_name):
        try:
            marker_id = int(topic_name.split('_')[-1])
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
        except Exception as e:
            self.get_logger().error(f'Error in marker_callback: {e}')

    def listener_callback(self, msg):
        agv = self.agv
        pmsg = Agvop()

        agv.reqid_ros2[2][0] = msg.reqid_s
        agv.reqid_ros2[2][1] = msg.seqid_s

        # estop msg primary message
        if msg.reqid_s == 1:
            agv.estop_server2 = True
            pmsg.op_state_a = 255

        # terminate program message
        elif msg.reqid_s == 4097:
            agv.will_terminate = True
            pmsg.op_state_a = 255

        elif not agv.thread_check_total:
            pmsg.op_state_a = 0
            agv.reqid_ros2[2][0] = 0
            agv.reqid_ros2[2][1] = 0

        # estop reset message treat
        elif agv.estop:
            # req:17 task reset with estop reset
            if msg.reqid_s == 17:
                agv.estop_fin_command_2server = True
                agv.estop_server2 = False
                pmsg.op_state_a = 255

                # param1 == 1 이면 task reset
                if msg.param1 == 1:
                    agv.reqid_ros2 = [[0, 0], [0, 0], [0, 0]]
                    agv.op_state = [0, 0, 0]
                    agv.task_clear = True
            else:
                pmsg.op_state_a = 0

        elif agv.reqid_ros2[0][0] == 0 and (msg.reqid_s == 513):
            # task assigned
            agv.is_assigned = True
            agv.op_id = msg.param[4]
            pmsg.op_state_a = 255

        elif agv.reqid_ros2[0][0] == 0 and (msg.reqid_s == 529):
            # assigned task clear
            agv.is_assigned = False
            agv.op_id = ""
            pmsg.op_state_a = 255

        elif agv.reqid_ros2[0][0] == 0 and (msg.reqid_s == 545):
            # battery id assign
            agv.battery_id = msg.param[4]
            pmsg.op_state_a = 255

        elif agv.reqid_ros2[0][0] == 0 and (msg.reqid_s == 561):
            # battery id clear
            agv.battery_id = ""
            agv.is_loaded = False
            pmsg.op_state_a = 255

        elif not agv.drv_srv:
            # servo off state msg
            if agv.reqid_ros2[0][0] == 0 and (msg.reqid_s == 9):
                # servo on command
                agv.reqid_ros2[0][0] = msg.reqid_s
                agv.reqid_ros2[0][1] = msg.seqid_s
                agv.op_state[0] = 1
                agv.param = [msg.param1, msg.param2, msg.param3, msg.param4, msg.param5, msg.param6]
                pmsg.op_state_a = 1

            elif agv.reqid_ros2[0][0] == 0 and (msg.reqid_s == 5):
                # cpos command
                agv.reqid_ros2[0][0] = msg.reqid_s
                agv.reqid_ros2[0][1] = msg.seqid_s
                agv.op_state[0] = 1
                agv.param = [msg.param1, msg.param2, msg.param3, msg.param4, msg.param5, msg.param6]
                pmsg.op_state_a = 1

            else:
                pmsg.op_state_a = 0
                agv.reqid_ros2[2][0] = 0
                agv.reqid_ros2[2][1] = 0

        # general command treatment when there is no assigned job in primary slot.
        elif agv.reqid_ros2[0][0] == 0 and (msg.reqid_s & 1):
            if (not agv.can_lift or not agv.was_home_top) and msg.reqid_s == 305:
                pmsg.op_state_a = 0
                agv.reqid_ros2[2][0] = 0
                agv.reqid_ros2[2][1] = 0

            elif not agv.can_move and msg.reqid_s == 257:
                pmsg.op_state_a = 0
                agv.reqid_ros2[2][0] = 0
                agv.reqid_ros2[2][1] = 0

            else:
                # primary slot is filled with the msg
                agv.reqid_ros2[0][0] = msg.reqid_s
                agv.reqid_ros2[0][1] = msg.seqid_s
                agv.op_state[0] = 1
                agv.param = [msg.param1, msg.param2, msg.param3, msg.param4, msg.param5, msg.param6]
                pmsg.op_state_a = 1

        # during operation, job command will be ignored.
        else:
            pmsg.op_state_a = 0
            agv.reqid_ros2[2][0] = 0
            agv.reqid_ros2[2][1] = 0

        pmsg.reqid_a = msg.reqid_s
        pmsg.seqid_a = msg.seqid_s

        self.publisher2_.publish(pmsg)

    def agvmonitoring_callback(self):
        msg = Amonitoring()
        msg.battery_id = self.agv.battery_id
        msg.is_loaded = self.agv.is_loaded
        msg.op_id = self.agv.op_id
        msg.is_assigned = self.agv.is_assigned
        msg.pin_state = self.agv.pin_state
        msg.lift_height = self.agv.lift_height
        msg.driving_state = self.agv.driving_state
        msg.lift_state = self.agv.lift_state_ros
        msg.t_pos = self.agv.t_pos
        msg.c_pos = self.agv.c_pos
        msg.stamp = self.get_clock().now().to_msg()
        msg.estop = self.agv.estop
        msg.error_a = self.agv.error
        msg.is_auto = self.agv.is_auto
        self.publisher_.publish(msg)

    def agv_res_auto(self):
        if self.agv.reqid_ros2[0][0] != 0:
            msg = Agvop()
            msg.reqid_a = self.agv.reqid_ros2[0][0]
            msg.seqid_a = self.agv.reqid_ros2[0][1]
            msg.op_state_a = self.agv.op_state[0]
            self.publisher2_.publish(msg)

            if msg.op_state_a == 255:
                self.agv_monitoring = False
                self.agv.reqid_ros2[0] = [0, 0]
                self.agv.reqid[0] = [0, 0]
                self.agv.op_state[0] = 0

    def agv_joint_update(self):
        msg = JointState()
        msg.x_pos = self.servoState.topplate_cpos[1]
        msg.y_pos = self.servoState.topplate_cpos[0]
        msg.yaw_pos = self.servoState.topplate_cpos[2]
        msg.lpin_pos = self.servoState.topplate_cpos[5]
        msg.lpin_rotate = self.servoState.topplate_cpos[3]
        msg.rpin_pos = self.servoState.topplate_cpos[6]
        msg.rpin_rotate = self.servoState.topplate_cpos[4]
        msg.lift_height = 0.0
        msg.lw_vel = 0.0
        msg.rw_vel = 0.0
        self.publisher3_.publish(msg)

    def lcam_listener_callback(self, msg):
        self.agv.lcam_hole_pos[0] = msg.x
        self.agv.lcam_hole_pos[1] = msg.y
        self.agv.lcam_hole_pos[2] = msg.z

    def rcam_listener_callback(self, msg):
        self.agv.rcam_hole_pos[0] = msg.x
        self.agv.rcam_hole_pos[1] = msg.y
        self.agv.rcam_hole_pos[2] = msg.z


def agv_planning_main(agv, servostate):
    rclpy.init(args=None)
    srv_sub = SrvreqSubscriber(agv, servostate)
    rclpy.spin(srv_sub)
    srv_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        from ..leadshine.agv_con import AGV
    except ImportError:
        class AGV:
            pass

    agv_planning_main(agv=AGV(), servostate=None)
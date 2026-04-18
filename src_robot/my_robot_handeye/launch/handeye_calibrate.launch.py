import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_base_frame     = 'mb_1'
    robot_effector_frame = 'lpin_1'   # 오른쪽 카메라면 'rpin_1'
    camera_frame         = 'camera_color_optical_frame'
    calibration_type     = 'eye_in_hand'
    marker_size          = 0.05       # ← 실측값 (단위: m)

    camera_topic      = '/camera/color/image_raw'
    camera_info_topic = '/camera/color/camera_info'

    # 1. ros2_aruco 마커 검출 노드
    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        name='aruco_node',
        output='screen',
        parameters=[{
            'marker_size':         marker_size,
            'aruco_dictionary_id': 'DICT_5X5_50',
            'image_topic':         camera_topic,
            'camera_info_topic':   camera_info_topic,
            'camera_frame':        camera_frame,
        }],
    )

    # 2. /aruco_markers → TF 변환 브릿지
    aruco_tf_bridge = Node(
        package='my_robot_handeye',
        executable='aruco_to_tf_bridge',
        name='aruco_to_tf_bridge',
        output='screen',
        parameters=[{
            'marker_frame_prefix': 'marker_',
        }],
    )

    # 3. easy_handeye2 캘리브레이션 GUI
    easy_handeye2_launch_dir = os.path.join(
        get_package_share_directory('easy_handeye2'), 'launch'
    )

    easy_handeye_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(easy_handeye2_launch_dir, 'calibrate.launch.py')
        ),
        launch_arguments={
            'calibration_type':      calibration_type,
            'name':                  'd435_eye_in_hand_calib',
            'robot_base_frame':      robot_base_frame,
            'robot_effector_frame':  robot_effector_frame,
            'tracking_base_frame':   robot_base_frame,   # eye_in_hand → 베이스 기준
            'tracking_marker_frame': 'marker_0',
            'freehand_robot_movement': 'true',
        }.items()
    )

    return LaunchDescription([
        aruco_node,
        aruco_tf_bridge,
        easy_handeye_node,
    ])
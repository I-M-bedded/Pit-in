# testbed_agv
testbed control code of PITIN

Requirements
1. full version package is developed under Ubuntu 22.04 lts and ROS2 Humble. python 3.10.12
2. For the convenience, Window based partial functions can be run. Automatically, run by checking os.
3. full package use pitin_msgs package for the ROS custom msgs. please build it.

Required packages.  
1. numpy : math lib
2. matplotlib : plot tools
3. pygame, evdev : xbox controller interface
4. requests : https protocol api (ros2 msgs via ros2-websocket bridge)

File discriptions
1. controller/evdev_controlthread.py : linux xbox controller interface code
2. contorller/ik.py                  : ik calculator. Simplified code for battery assembly. 
3. fastech/protocol.py               : fastech servo control api 
4. fastech/servo_state.py            : Joint state publisher
5. fastech/servolist.py              : kind of header for protocol
6. ros2/interface.py                 : default ros2 interface for the real robot
7. ros2/test.js                      : Websocket command script (Require node js, roslibjs)
8. leadshine/agv_con.py              : agv controller. which has almost non necessary features. (original robot is developed from agv. so it has many essential variables.)
9. leadshine/agv_dummy.py            : agv controller dummy file. (clean version)
10. install.py                       : install required package
11. robot.py                         : robot controller, include some non-necessary functions from robot code. not yet fixed.
12. test.py                          : clean code for control 7 motor control and comm via ros2 msgs. Main.
13. controller/joy.py                : window xbox controller interface code.
14. pitin_msgs.zip                   : ROS2 custom msg package for pitin agv. -> : ros 메시지로 ros관련 워크스페이스에 넣어서 빌드할 필요 존재!
15. vision/calibration               : Camera sensor calibration codes (Intrinsic, Extrinsic)

Xbox controller can be used for the manual control.


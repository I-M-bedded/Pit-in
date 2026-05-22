# Fisheye Camera Hand-Eye Calibration and Hole Position Estimation

이 문서는 기존 Pit-in 비전 파이프라인을 fisheye camera에 맞게 바꾸는 방법을 정리한다.

기존 구조는 다음 파일을 기준으로 한다.

- `vision/calibration/calibrate_fisheye.py`: fisheye intrinsic calibration
- `vision/calibration/sturdecam31_fisheye_intrinsics.yaml`: OpenCV fisheye K/D 결과
- `src_robot/my_robot_handeye/launch/handeye_calibrate.launch.py`: ArUco + easy_handeye2 hand-eye calibration
- `vision/yolo/hole_pose_estimator.py`: YOLO-Pose keypoint 기반 hole camera-frame pose 추정
- `vision/vision_node.py`: camera-frame pose를 robot/world frame으로 변환하여 `/cam0`, `/cam1` 발행
- `vision/yolo/HOLE_POSE_ESTIMATOR.md`: 기존 pinhole 기준 tiered hole pose 방법론


## 핵심 결론

Fisheye camera에서 가장 중요한 수정점은 "net의 2-D 출력 pixel"을 곧바로 pinhole 식에 넣지 않는 것이다.

기존 pinhole 식:

```text
u = fx * X / Z + cx
v = fy * Y / Z + cy
```

은 fisheye raw image에서는 성립하지 않는다. 따라서 hole pose 계산 전에 반드시 다음 둘 중 하나를 선택해야 한다.

1. Raw fisheye image를 rectified pinhole image로 변환한 뒤 기존 pipeline을 거의 그대로 사용한다.
2. Net이 raw fisheye image에서 학습/추론한다면, 추론된 keypoint pixel을 `cv2.fisheye.undistortPoints`로 normalized ray 또는 rectified pixel로 바꾼 뒤 PnP/geometry에 사용한다.

Pit-in에서는 net이 "fisheye camera로 학습된 raw image network"라고 가정하므로 2번을 기본안으로 둔다. 단, hand-eye calibration은 안정성을 위해 rectified image를 만들어 기존 `easy_handeye2` 흐름을 유지하는 방식을 권장한다.


## 1. Fisheye Intrinsic Calibration

먼저 camera intrinsic을 fisheye model로 구한다.

```bash
python3 vision/calibration/calibrate_fisheye.py \
  --images "vision/calibration/calib_img/*.jpg" \
  --board-cols 9 \
  --board-rows 6 \
  --square-size 0.025 \
  --output vision/calibration/sturdecam31_fisheye_intrinsics.yaml \
  --preview-dir vision/calibration/fisheye_preview
```

결과 파일에는 다음 값이 있어야 한다.

- `camera_model: opencv_fisheye`
- `K`: raw fisheye image의 intrinsic matrix
- `D`: OpenCV fisheye distortion coefficients, shape `(4, 1)`
- `K_undistort`: rectified pinhole image에 사용할 virtual intrinsic
- `mean_reprojection_error_px`: 보정 품질 확인용

현재 repo의 `sturdecam31_fisheye_intrinsics.yaml`은 평균 reprojection error가 약 `0.323 px`라서 hand-eye와 pose estimation에 사용할 수 있는 수준이다.


## 2. Fisheye Hand-Eye Calibration

### 2.1 권장 방식: rectified ArUco image로 easy_handeye2 사용

기존 `handeye_calibrate.launch.py`는 `ros2_aruco`가 `/camera/color/image_raw`와 `/camera/color/camera_info`를 받아 marker pose를 만들고, `easy_handeye2`가 그 TF를 이용해 eye-in-hand calibration을 수행한다.

문제는 `ros2_aruco`와 일반 `CameraInfo` 기반 ArUco pipeline이 fisheye raw distortion을 정확히 처리하지 못할 수 있다는 점이다. 따라서 calibration 때는 다음 가상 pinhole stream을 만들어 쓰는 것이 안전하다.

```text
raw fisheye image
  -> cv2.fisheye.initUndistortRectifyMap(K, D, R=I, P=K_undistort)
  -> rectified image
  -> CameraInfo(K=K_undistort, D=zeros)
  -> ros2_aruco
  -> aruco_to_tf_bridge
  -> easy_handeye2
```

이 방식의 장점은 다음과 같다.

- `easy_handeye2`와 `ros2_aruco`를 큰 수정 없이 그대로 쓸 수 있다.
- ArUco corner와 PnP가 pinhole image 위에서 동작하므로 왜곡 오차가 줄어든다.
- Rectification에서 `R=I`를 쓰면 camera optical frame은 물리 camera frame과 같은 축으로 둘 수 있다. 즉 hand-eye 결과는 기존처럼 `camera_color_optical_frame` 기준으로 해석하면 된다.

수행 절차:

1. `calibrate_fisheye.py`로 `K`, `D`, `K_undistort`를 만든다.
2. Fisheye image를 rectified image topic으로 publish한다.
3. Rectified image의 `CameraInfo`는 `K=K_undistort`, `D=[0, 0, 0, 0, 0]`, distortion model은 `plumb_bob` 또는 empty distortion으로 둔다.
4. `handeye_calibrate.launch.py`의 `camera_topic`, `camera_info_topic`을 rectified topic으로 바꾼다.
5. 기존과 같이 다음 TF 연결을 확인한다.

```bash
ros2 run tf2_ros tf2_echo mb_1 marker_0
ros2 run tf2_ros tf2_echo mb_1 lpin_1
ros2 run tf2_ros tf2_echo mb_1 camera_color_optical_frame
```

6. `easy_handeye2` GUI에서 충분히 다른 robot pose를 수집한다.

좋은 sample 조건:

- marker가 화면 중앙뿐 아니라 좌/우/상/하에 분포하도록 움직인다.
- yaw만 바꾸지 말고 가능한 범위에서 pitch/roll 성분도 조금씩 포함한다.
- 같은 위치에서 아주 작은 움직임만 반복하지 않는다.
- marker pose reprojection이 튀는 sample은 제거한다.

최종 결과는 다음 transform이다.

```text
T_effector_camera
```

Pit-in에서는 left camera이면 보통 `lpin_1 -> camera_color_optical_frame`, right camera이면 `rpin_1 -> camera_color_optical_frame`로 저장한다. 이 transform의 회전은 `vision/vision_node.py`의 `_CAM_MOUNT_R`, translation은 `_CAM_OFFSET`에 반영하거나, 가능하면 TF에서 직접 읽도록 바꾸는 것이 좋다.


### 2.2 대안: raw fisheye corner로 직접 marker TF 생성

Rectified image topic을 만들기 어렵다면 ArUco corner detection은 raw image에서 하고, marker pose 계산만 fisheye 보정 좌표에서 수행할 수 있다.

절차:

1. Raw image에서 ArUco corner pixel `p_raw = [u, v]`를 검출한다.
2. Corner들을 normalized ray로 변환한다.

```python
pts_norm = cv2.fisheye.undistortPoints(
    pts_raw.reshape(-1, 1, 2).astype(np.float64),
    K,
    D,
    R=np.eye(3),
    P=None,
).reshape(-1, 2)
```

3. `solvePnP`에는 normalized point와 identity camera를 넣는다.

```python
K_norm = np.eye(3, dtype=np.float64)
D_zero = np.zeros(4, dtype=np.float64)
ok, rvec, tvec = cv2.solvePnP(
    marker_points,
    pts_norm,
    K_norm,
    D_zero,
    flags=cv2.SOLVEPNP_IPPE_SQUARE,
)
```

4. 이렇게 얻은 `marker -> camera` pose를 TF로 publish하고, `easy_handeye2`에는 기존처럼 marker TF만 공급한다.

이 방식은 `ros2_aruco`를 대체하는 custom node가 필요하지만, raw fisheye 좌표를 직접 다루므로 모델 가정이 명확하다.


## 3. Fisheye Net으로 Hole Position을 구하는 방법

가정:

- Net은 fisheye camera의 raw image로 학습되어 있다.
- Net 출력은 기존 YOLO-Pose와 동일하게 class, bbox, `kp_0`, boundary keypoint 8개, confidence를 준다.
- 물리 target geometry는 기존 `GEOMETRY_SPECS`와 `_BOARD_OFFSETS_M`를 그대로 사용한다.
- 최종 출력은 기존처럼 center hole의 robot/world-frame 3-D position이다.


### 3.1 전체 runtime flow

```text
raw fisheye frame
  -> fisheye-trained net inference
  -> raw pixel keypoints
  -> fisheye undistortPoints
  -> normalized/rectified 2-D keypoints
  -> solvePnP or board geometry
  -> t_hole_camera
  -> hand-eye/FK transform
  -> x_hole_world
  -> WorldTracker EMA
  -> publish /cam0 or /cam1
```

기존 `WorldTracker`와 macro-micro PBVS는 world 좌표를 받으므로 바꿀 필요가 거의 없다. 변경은 camera image geometry 안쪽, 즉 `HolePoseEstimator`와 `VisionNode`의 projection/undistortion 부분에 집중된다.


### 3.2 Camera model wrapper 추가

기존 `HolePoseEstimator`는 `self.K`, `self.dist`를 pinhole distortion으로 보고 다음을 직접 수행한다.

- `cv2.solvePnP(obj, img, K, dist)`
- `cv2.projectPoints(obj, rvec, tvec, K, dist)`
- `_project_cam_to_pixel()`에서 pinhole 수식 직접 사용

Fisheye에서는 이 부분을 camera model wrapper로 분리하는 것이 좋다.

필요 기능:

```text
undistort_points(raw_px) -> normalized_xy or rectified_px
project_points(points_cam) -> raw fisheye pixel
solve_pose(object_points, raw_image_points) -> rvec, tvec
```

권장 구현:

- 내부 PnP 계산은 normalized coordinate에서 한다.
- overlay, prior projection, pixel consistency는 raw fisheye pixel로 다시 project해서 비교한다.

정리하면:

```python
pts_norm = cv2.fisheye.undistortPoints(raw_px, K, D, R=I, P=None)
solvePnP(object_points, pts_norm, I, 0)
cv2.fisheye.projectPoints(points_3d, rvec, tvec, K, D)
```


### 3.3 Tier별 수정

기존 `HOLE_POSE_ESTIMATOR.md`의 tier 구조는 유지하되, 각 tier의 2-D 좌표 처리를 바꾼다.

#### Tier 1: center inner + outer pair

기존:

- inner/outer center hole이 같이 잡히면 신뢰도를 계산한다.
- inner circle ellipse에서 conic pose를 우선 사용한다.
- 실패하면 PnP로 fallback한다.

Fisheye 수정:

- pair reliability 계산은 raw pixel 거리로 해도 된다. 같은 image 안의 detector 안정성 판단이기 때문이다.
- pose 계산은 raw pixel을 바로 쓰지 말고 boundary keypoint를 undistort한 뒤 PnP를 우선 사용한다.
- raw fisheye image에서 fit한 ellipse를 그대로 conic back-projection에 넣는 것은 피한다. Fisheye distortion 후의 ellipse는 pinhole conic 가정을 만족하지 않는다.
- conic branch를 쓰고 싶다면 boundary point를 rectified pinhole pixel로 바꾼 뒤 ellipse를 다시 fit해야 한다.

권장 Tier 1 출력:

```text
source = "fisheye_pnp_pair"
tvec_m = inner center-hole pose from undistorted keypoint PnP
center_xy = raw fisheye projection of tvec_m
```


#### Tier 2a: single center detection

기존:

- inner center만 있으면 circle conic recovery
- outer center만 있으면 slot PnP

Fisheye 수정:

- inner-only에서도 conic depth approximation은 기본적으로 끈다.
- inner/outer 모두 keypoint PnP를 사용한다.
- boundary keypoint가 부족하면 `kp_0`와 남은 boundary를 섞되, 최소 4점 이상일 때만 PnP를 수행한다.
- PnP reprojection error는 raw fisheye pixel 기준으로 다시 계산해서 confidence에 반영한다.

권장 confidence:

```text
conf_2a = detector_conf * keypoint_support * reprojection_gain
reprojection_gain = 1 / (1 + raw_reproj_error_px / 2)
```


#### Tier 2b: guide holes + row geometry fallback

기존 Tier 2b의 3-D row fitting, duplicate clustering, board offset matching은 fisheye와 직접 관련이 없다. 따라서 대부분 유지한다.

수정할 부분:

- 각 detection의 3-D anchor를 만들 때 undistorted keypoint PnP를 사용한다.
- `_project_cam_to_pixel()`은 pinhole 직접식 대신 fisheye projection을 사용한다.
- prior 비교도 raw fisheye projection과 raw detector pixel 사이에서 수행한다.

즉, 3-D 단계는 그대로 두고 2-D <-> 3-D camera projection 함수만 fisheye-aware로 바꾼다.


### 3.4 World-frame 변환

Hole pose estimator가 반환하는 값은 여전히 camera frame 기준이다.

```text
t_hole_cam = [X, Y, Z]^T
```

Hand-eye/FK로 얻은 camera pose가 다음과 같을 때:

```text
R_cam_world = camera frame -> robot/world frame rotation
p_cam_world = camera origin in robot/world frame
```

기존 `WorldTracker`와 동일하게 변환한다.

```text
x_hole_world = p_cam_world + R_cam_world * t_hole_cam
```

따라서 fisheye 변경은 hand-eye 결과의 정확도를 높이고, `t_hole_cam`을 정확히 만들기 위한 camera model 수정에 집중된다. PBVS controller가 받는 `/cam0`, `/cam1`의 의미는 계속 world-frame hole position이다.


## 4. 기존 코드에서 바꿀 부분

### 4.1 `vision/vision_node.py`

현재는 RealSense intrinsic을 스트림에서 읽고 pinhole처럼 사용한다.

수정 방향:

- fisheye camera 사용 시 `sturdecam31_fisheye_intrinsics.yaml`에서 `K`, `D`, `K_undistort`, `camera_model`을 읽는다.
- `HolePoseEstimator(..., camera_model="opencv_fisheye")`처럼 model type을 넘긴다.
- `_compute_prior()`에서 pinhole 직접 투영 대신 estimator/camera model의 `project_point()`를 호출한다.


### 4.2 `vision/yolo/hole_pose_estimator.py`

수정 방향:

- `load_intrinsics()`가 OpenCV YAML의 `opencv_fisheye`, `K_undistort`를 읽도록 확장한다.
- `solve_pose()`를 camera model별로 분기한다.
- `_project_cam_to_pixel()`을 camera model projection으로 교체한다.
- raw fisheye ellipse 기반 `conic_to_pose()`는 기본 비활성화한다.
- conic을 유지하려면 `undistortPoints(..., P=K_undistort)`로 boundary를 rectified pixel로 변환한 뒤 ellipse fit부터 다시 한다.


### 4.3 `src_robot/my_robot_handeye/launch/handeye_calibrate.launch.py`

수정 방향:

- calibration 때 사용할 image topic을 raw가 아니라 rectified topic으로 바꾼다.

예:

```python
camera_topic = "/fisheye/rect/image"
camera_info_topic = "/fisheye/rect/camera_info"
camera_frame = "camera_color_optical_frame"
```

또는 raw fisheye ArUco pose node를 직접 만들 경우:

- `ros2_aruco` node를 빼고 custom fisheye ArUco pose publisher를 넣는다.
- `aruco_to_tf_bridge`와 `easy_handeye2`는 그대로 둔다.


### 4.4 `vision/calibration/camera.py`

이 파일은 현재 RealSense + 일반 `solvePnP`로 marker pose를 publish한다. Fisheye camera로 재사용하려면 다음 둘 중 하나로 바꾼다.

- rectified image를 입력으로 쓰고 `K_undistort`, `D=0`으로 solvePnP
- raw image에서 corner를 찾은 뒤 `cv2.fisheye.undistortPoints` + `K=I`, `D=0`으로 solvePnP


## 5. 검증 방법

### 5.1 Intrinsic 검증

- `mean_reprojection_error_px < 0.5 px`이면 우선 사용 가능
- 화면 가장자리 chessboard pose의 error가 중앙보다 지나치게 크면 calibration image를 추가한다.

### 5.2 Hand-eye 검증

고정된 ArUco marker를 여러 robot pose에서 관측했을 때, marker의 world 좌표가 거의 같은 위치로 모여야 한다.

검증 기준:

```text
mean world XY spread <= 2~3 mm: 좋음
mean world XY spread <= 5 mm: 사용 가능
그 이상: intrinsic, marker size, frame id, camera offset, sample 다양성 재확인
```

현재 repo의 `vision/calibration/simple_hand_eyecali.py`는 이런 "같은 marker가 world에서 얼마나 모이는지"를 보는 planar hand-eye tuning 방식이다. Fisheye hand-eye 결과도 같은 방식으로 sanity check할 수 있다.


### 5.3 Hole pose 검증

1. Fisheye net inference 결과를 raw image에 overlay한다.
2. 각 detection에 대해 raw reprojection error를 기록한다.
3. 같은 hole을 여러 robot pose에서 봤을 때 `/cam0`, `/cam1` world position이 같은 좌표로 모이는지 확인한다.
4. PBVS에 넣기 전에는 `WorldTracker` raw/filt 값을 같이 log해서 튀는 frame을 확인한다.

권장 log:

```text
source
confidence
raw_tvec_cam
raw_world
filtered_world
raw_reprojection_error_px
num_valid_keypoints
```


## 6. 최종 권장안

Hand-eye calibration:

- Fisheye intrinsic은 `calibrate_fisheye.py`로 구한다.
- Calibration용 ArUco는 rectified pinhole image로 변환해서 기존 `easy_handeye2`를 사용한다.
- 결과 transform은 `vision_node.py`의 camera mount transform 또는 TF 기반 camera pose 계산에 반영한다.

Hole position estimation:

- Net은 raw fisheye image에서 추론한다.
- Net이 낸 raw keypoint pixel은 pose 계산 전에 `cv2.fisheye.undistortPoints`로 normalized ray로 변환한다.
- Pose는 conic depth approximation보다 PnP를 우선한다.
- 기존 Tier 1/2a/2b 구조와 WorldTracker는 유지하되, projection/back-projection 함수만 fisheye-aware로 교체한다.
- 최종 `/cam0`, `/cam1`은 계속 robot/world-frame의 hole center position `[x, y, z]`로 publish한다.


## 7. 구현 시작점

현재 구현은 "raw fisheye conic을 억지로 푸는 방향"이 아니라, 더 쉬운 문제인 "pixel을 undistort한 평면 기준에서 IPPE를 푸는 방향"으로 잡았다. Top-level 구조는 fisheye/pinhole 공통이다.

추가된 runtime 경로:

```text
HolePoseEstimator
  -> _estimate_center_high()
      pinhole: center conic -> center IPPE fallback
      fisheye: center IPPE only
  -> _estimate_board_ippe()
      weak/missing center: board-template assignment -> board-plane IPPE
  -> _tier2_line_fitting()
      legacy per-detection anchor + 3-D row fitting fallback
```

핵심 차이:

- 중앙 신뢰도가 높으면 중앙 target을 먼저 사용한다.
- pinhole 중앙 target은 conic이 가능하면 conic을 우선 사용하고, 실패하면 원 keypoint IPPE를 사용한다.
- fisheye 중앙 target은 conic 없이 원 keypoint IPPE만 사용한다.
- 중앙 신뢰도가 낮으면 개별 hole 3-D anchor를 먼저 만들지 않고, 관찰된 detection들을 board template에 할당한 뒤 board 전체를 IPPE로 푼다.
- fisheye PnP는 `cv2.fisheye.undistortPoints(..., P=None)`로 raw pixel을 normalized point로 바꾼 뒤 `K=I`, `D=0`, `SOLVEPNP_IPPE`만 사용한다.
- board IPPE는 pinhole/fisheye 모두 `SOLVEPNP_IPPE`만 사용한다.
- reprojection error와 prior 비교는 각 camera model의 projection 함수로 raw image pixel에 다시 투영해서 계산한다.

Runtime 예:

```bash
python3 vision/vision_node.py \
  --weights vision/result/weight/yolo_baseline_n.pt \
  --intrinsics vision/calibration/sturdecam31_fisheye_intrinsics.yaml \
  --camera-model auto \
  --side left
```

오프라인 이미지 평가 예:

```bash
python3 vision/yolo/hole_pose_estimator.py \
  --source <image_or_dir> \
  --yolo-weights <fisheye_trained_pose_weight.pt> \
  --intrinsics vision/calibration/sturdecam31_fisheye_intrinsics.yaml \
  --camera-model auto \
  --save-json \
  --save-vis
```


## 8. Low-Center Case 재설계 방향

정리하면 다음처럼 나누는 것이 좋다.

```text
center confidence high
  -> center target keypoints로 IPPE

center confidence low, guide/outer holes enough
  -> line 자체를 PnP하는 것이 아니라
  -> guide holes를 board template에 robust matching
  -> matched planar correspondences 전체로 board pose IPPE
  -> board origin을 center hole position으로 사용
```

즉 "line에 대해 IPPE"가 아니라 "line으로 template assignment를 안정화한 뒤, board plane 전체에 대해 IPPE"를 푸는 구조가 맞다. IPPE는 2-D line만으로는 pose를 풀 수 없고, planar 3-D point와 image point correspondences가 필요하다.


### 8.1 왜 Hough보다 RANSAC/MSAC 쪽이 나은가

Hough transform은 edge image에서 긴 직선을 찾을 때 강하다. 하지만 여기서는 검출기가 이미 hole 후보를 sparse하게 몇 개만 준다. 실제 입력은 edge map이 아니라 다음과 같은 sparse observations다.

```text
hole candidate center
boundary keypoints
class/confidence
ellipse/slot shape cue
```

따라서 Hough보다 RANSAC/MSAC 계열이 더 자연스럽다.

- Hough: 이미지 전체 edge line 검출에는 좋지만, hole 2~5개 sparse point에는 과하다.
- RANSAC/MSAC: outlier hole detection, duplicate detection, missing hole이 있어도 template hypothesis를 robust하게 고를 수 있다.
- Weighted RANSAC/MSAC: detector confidence와 keypoint support를 cost에 바로 넣을 수 있다.

추천은 `weighted MSAC + template assignment + IPPE`다. RANSAC처럼 inlier/outlier를 고르되, hard threshold만 쓰지 않고 residual을 truncated quadratic cost로 누적한다.


### 8.2 Pinhole/Fisheye 공통화 방법

공통 layer는 pixel을 normalized bearing coordinate로 바꾸는 것이다.

Pinhole:

```text
x_n = (u - cx) / fx
y_n = (v - cy) / fy
```

Fisheye:

```python
xy_norm = cv2.fisheye.undistortPoints(raw_px, K, D, P=None)
```

이후 알고리즘은 pinhole/fisheye가 거의 같다.

```text
raw pixel
  -> camera_model.undistort_points(...)
  -> normalized image points
  -> robust board-template matching
  -> IPPE with K=I, D=0
  -> camera_model.project_points(...) for raw-pixel reprojection score/debug
```

즉 fisheye만 특별 취급하는 부분은 `undistort_points`와 `project_points`뿐이다. Line matching, assignment, IPPE, confidence 계산은 공통으로 쓸 수 있다.


### 8.3 Center High Case

중앙 신뢰도가 높으면 가장 단순하게 간다.

사용 후보:

- `center_hole_inner`가 있으면 inner target의 center + boundary keypoints
- `center_hole_outer`가 같이 있으면 center pair reliability로 gate
- raw fisheye conic은 사용하지 않음

절차:

```text
1. center target keypoints 선택
2. keypoints를 normalized coordinate로 변환
3. planar target template과 대응
4. IPPE
5. center/hole canonical offset 적용
6. raw-pixel reprojection error로 confidence 보정
```


### 8.4 Center Low Case: Robust Board IPPE

중앙 hole 신뢰도가 낮으면 guide holes와 outer holes를 이용해서 board pose를 직접 푼다.

기존 Case 2b는 detection마다 작은 target PnP를 먼저 풀고, 그 3-D anchor들을 line fitting한다. 이 방식은 fallback으로 쓸 수 있지만, 더 좋은 1순위 fallback은 다음이다.

```text
1. 각 detection에서 reliable point를 뽑는다.
   - kp0 center
   - boundary keypoints
   - class: inner/outer, guide/center

2. image에서 hole candidate center들을 normalized coordinate로 변환한다.

3. candidate centers에 weighted line RANSAC/MSAC를 수행한다.
   - sample: candidate center 2개
   - model: normalized image plane의 2-D line
   - residual: point-to-line distance
   - weight: detection_conf * keypoint_support

4. line 위 projection 순서로 candidates를 정렬한다.

5. board template offset subset을 enumerate한다.
   - template: [-0.22, -0.08, 0.0, 0.08, 0.22] m
   - visible count k에 맞는 subset 선택
   - class 정보로 center/guide 가능 위치를 gate

6. 각 assignment마다 planar correspondences를 만든다.
   - 최소 대응은 hole center 4점 이상
   - 더 좋게는 각 hole의 boundary keypoints까지 template local geometry로 포함

7. IPPE로 board pose를 푼다.
   - object point: board coordinate의 hole center/boundary
   - image point: normalized coordinate
   - K=I, D=0

8. raw-pixel reprojection error, template residual, class consistency로 score를 계산한다.

9. best board pose의 board origin `[0, 0, 0]`을 camera frame으로 변환해서 center hole position으로 사용한다.
```

이 방식의 장점:

- guide hole을 개별 3-D anchor로 만들지 않고, board pose 하나를 global하게 푼다.
- fisheye/pinhole 모두 normalized coordinate 이후 동일한 solver를 쓴다.
- center가 안 보여도 board origin을 직접 복원할 수 있다.
- duplicate/outlier는 MSAC score에서 자연스럽게 밀린다.


### 8.5 Scoring

각 hypothesis score는 다음처럼 둔다.

```text
score =
  raw_reprojection_rms_px
  + lambda_line * line_residual_norm
  + lambda_scale * abs(template_scale - 1)
  + lambda_class * class_mismatch
  + lambda_prior * prior_distance
```

단, 최종 reprojection error는 normalized coordinate가 아니라 raw pixel 기준으로 다시 계산하는 것이 좋다. 실제 detector output이 raw pixel이고, debug overlay도 raw image이기 때문이다.

Confidence는 score를 역변환한다.

```text
reproj_gain = 1 / (1 + raw_reprojection_rms_px / 2)
line_gain   = 1 / (1 + line_residual_norm / tau)
conf        = mean_detection_conf * reproj_gain * line_gain
```


### 8.6 최종 우선순위

권장 fallback 순서는 다음이다.

```text
Tier F1: center high
  center target IPPE

Tier F2: center low, enough board observations
  weighted MSAC line + board-template assignment + board IPPE

Tier F3: board IPPE 실패
  기존 per-detection anchor + 3-D weighted row fitting

Tier F4: 실패
  no update, WorldTracker confidence decay
```

이렇게 두면 fisheye와 pinhole 모두 같은 "normalized point + planar IPPE" 구조를 공유하고, camera model 차이는 projection wrapper 안에만 남는다.

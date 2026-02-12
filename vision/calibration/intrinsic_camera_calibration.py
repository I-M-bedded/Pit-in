import glob, cv2, numpy as np

# ====== 이미지 경로 ======
files = sorted(glob.glob("/home/cora/Pictures/*.png"))
assert len(files) > 0, "PNG가 없습니다. 경로/확장자 확인하세요."

# ====== ArUco 설정 ======
DICT = cv2.aruco.DICT_4X4_50
marker_ids = [3, 4]

L = 0.05
g = 0.063
d = L + g

marker_centers = {
    3: np.array([0.0, 0.0, 0.0], dtype=np.float64),
    4: np.array([0.0,   d, 0.0], dtype=np.float64),
}

def marker_object_corners(center_xyz, L):
    cx, cy, cz = center_xyz
    h = L/2.0
    return np.array([
        [cx - h, cy + h, cz],
        [cx + h, cy + h, cz],
        [cx + h, cy - h, cz],
        [cx - h, cy - h, cz],
    ], dtype=np.float64)

# ====== 여기 K, D는 realsense-viewer에서 읽은 값으로 넣어야 함 ======
fx, fy, ppx, ppy = 907.788208007812, 906.967651367188, 649.08154296875, 362.698974609375 
k1, k2, p1, p2, k3 = 0, 0, 0, 0, 0              

K = np.array([[fx, 0,  ppx],
              [0,  fy, ppy],
              [0,  0,  1]], dtype=np.float64)
D = np.array([k1, k2, p1, p2, k3], dtype=np.float64)

# ====== OpenCV ArUco 호환 Detector ======
aruco_dict = cv2.aruco.getPredefinedDictionary(DICT)

if hasattr(cv2.aruco, "DetectorParameters"):
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)
    def detect(gray):
        return detector.detectMarkers(gray)
else:
    params = cv2.aruco.DetectorParameters_create()
    def detect(gray):
        return cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)

errs = []
for f in files:
    img = cv2.imread(f)
    if img is None:
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    corners_list, ids, _ = detect(gray)
    if ids is None:
        continue
    ids = ids.flatten().tolist()
    if not all(mid in ids for mid in marker_ids):
        continue

    img_pts, obj_pts = [], []
    for mid in marker_ids:
        idx = ids.index(mid)
        c2d = corners_list[idx].reshape(4,2).astype(np.float64)
        c3d = marker_object_corners(marker_centers[mid], L)
        img_pts.append(c2d); obj_pts.append(c3d)

    img_pts = np.vstack(img_pts)
    obj_pts = np.vstack(obj_pts)

    ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        continue

    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, D)
    proj = proj.reshape(-1,2)

    e = img_pts - proj
    per_point = np.linalg.norm(e, axis=1)
    rms = float(np.sqrt(np.mean(per_point**2)))
    mean = float(np.mean(per_point))
    errs.append((f, rms, mean))

if len(errs) == 0:
    print("두 마커가 동시에 검출된 프레임이 없습니다. dict/ID/조명/거리/마커가 화면에 같이 들어오는지 확인하세요.")
else:
    rms_all = sum(x[1] for x in errs)/len(errs)
    mean_all = sum(x[2] for x in errs)/len(errs)
    print(f"Used frames: {len(errs)}/{len(files)}")
    print(f"Avg RMS reprojection error:  {rms_all:.3f} px")
    print(f"Avg mean reprojection error: {mean_all:.3f} px")

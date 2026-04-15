import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
import os

class UnitAdjustedWingCalibration:
    def __init__(self, offset_vector):
        """
        Args:
            offset_vector (list or np.array): [x, y, z] (단위: cm)
        """
        self.offset_t = np.array(offset_vector)

    def load_data(self, csv_path):
        df = pd.read_csv(csv_path)
        # 유효 데이터 필터링
        df = df[df['m1_id'] != "N/A"].reset_index(drop=True)

        # 1. 로봇 포즈 (이미 cm 단위이므로 그대로 로드)
        robot_pos = df[['robot_x', 'robot_y']].to_numpy()
        robot_pos_3d = np.hstack([robot_pos, np.zeros((len(robot_pos), 1))])
        
        # 2. 로봇 회전 (Radian 단위 그대로 사용)
        robot_th = df['robot_th'].to_numpy()
        robot_rot_matrices = []
        for th in robot_th:
            # 날개 회전축(Z축) 기준 회전 행렬 생성
            r = R.from_euler('z', th, degrees=False).as_matrix()
            robot_rot_matrices.append(r)
        robot_rot_matrices = np.array(robot_rot_matrices)

        # 3. 마커 데이터 (m -> cm 변환 필수)
        m1_data = df[['m1_x', 'm1_y', 'm1_z']].to_numpy() * 100.0
        m1_ids = df['m1_id'].to_numpy()
        m2_data = df[['m2_x', 'm2_y', 'm2_z']].to_numpy() * 100.0
        m2_ids = df['m2_id'].to_numpy()

        return robot_pos_3d, robot_rot_matrices, m1_data, m1_ids, m2_data, m2_ids

    def _error_function(self, params, robot_pos, robot_rot, m1_data, m1_ids, m2_data, m2_ids, swap_axes=False):
        target_R = R.from_rotvec(params).as_matrix()
        all_residuals = []

        # 
        for m_data, m_ids in [(m1_data, m1_ids), (m2_data, m2_ids)]:
            valid_mask = ~pd.isna(m_ids)
            if not np.any(valid_mask): continue
            
            v_robot_pos = robot_pos[valid_mask]
            v_robot_rot = robot_rot[valid_mask]
            v_m_data = m_data[valid_mask].copy()

            # 사용자가 발견한 현상 반영: 로봇 X <-> 마커 Y 대응 시 스왑
            if swap_axes:
                v_m_data[:, [0, 1]] = v_m_data[:, [1, 0]]

            # 1. 카메라 좌표계를 Hand-Eye Matrix로 회전
            rotated_cam = (target_R @ v_m_data.T).T
            
            # 2. EE Offset(날개-카메라 거리) 적용 후 로봇 좌표계로 변환
            offset_added = rotated_cam - self.offset_t
            term_transformed = np.einsum('nij,nj->ni', v_robot_rot, offset_added)
            
            # 3. 월드 좌표계 상의 XY 위치 추정 (Z 제외)
            p_world_xy = (v_robot_pos + term_transformed)[:, :2]
            
            # 4. 각 마커 지점들의 평균으로부터의 거리(분산) 최소화
            center_xy = np.mean(p_world_xy, axis=0)
            all_residuals.append((p_world_xy - center_xy).flatten())

        return np.concatenate(all_residuals)

    def solve(self, csv_path, swap_axes=False):
        data = self.load_data(csv_path)
        robot_pos, robot_rot, m1_data, m1_ids, m2_data, m2_ids = data
        
        initial_guess = np.array([0.0, 0.0, 0.0])
        mode_label = "축 스왑(X<->Y)" if swap_axes else "표준(Standard)"
        
        result = least_squares(
            self._error_function, 
            initial_guess, 
            args=(robot_pos, robot_rot, m1_data, m1_ids, m2_data, m2_ids, swap_axes),
            method='trf'
        )

        if result.success:
            final_R = R.from_rotvec(result.x).as_matrix()
            euler = R.from_rotvec(result.x).as_euler('xyz', degrees=True)
            res_vectors = result.fun.reshape(-1, 2)
            mean_xy_err = np.mean(np.linalg.norm(res_vectors, axis=1))
            
            print(f"\n[{mode_label} 모드 결과]")
            print(f"평균 XY 오차: {mean_xy_err:.4f} cm")
            print(f"최종 Euler 각도(XYZ deg): {euler}")
            return mean_xy_err, final_R, euler
        return 1e9, None, None

if __name__ == "__main__":
    # 물리적 오프셋 (날개 중심 기준 카메라 위치, cm)
    known_offset_cm = [5.7, 2.9, -21.51] 
    
    calibrator = UnitAdjustedWingCalibration(offset_vector=known_offset_cm)
    csv_file = "Pit-in/vision/Calibration_data.csv"

    print("🚀 단위를 보정한 비교 최적화를 시작합니다...")
    err1, rot1, eul1 = calibrator.solve(csv_file, swap_axes=False)
    err2, rot2, eul2 = calibrator.solve(csv_file, swap_axes=True)

    print("\n" + "★" * 50)
    if err2 < err1:
        print(f"최종 선택: [축 스왑(X<->Y) 모드]가 압도적으로 정확합니다!")
        print(f"최종 평균 오차: {err2:.4f} cm")
        print(f"최종 회전 행렬 (E_R_C):\n{rot2}")
    else:
        print(f"최종 선택: [표준 모드]가 더 정확합니다.")
        print(f"최종 평균 오차: {err1:.4f} cm")
        print(f"최종 회전 행렬 (E_R_C):\n{rot1}")
    print("★" * 50)
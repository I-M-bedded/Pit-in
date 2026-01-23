import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares

class HandEyeCalibration:
    def __init__(self, offset_vector):
        """
        Args:
            offset_vector (list or np.array): [x, y, z] 
            * 주의: 반드시 cm 단위로 입력해야 합니다. (로봇 좌표계와 동일하게)
        """
        self.offset_t = np.array(offset_vector)

    def load_data(self, csv_path):
        df = pd.read_csv(csv_path)
        
        # 데이터 유효성 검사
        if df.isnull().values.any():
            print(f"[Warning] CSV에 결측치(NaN)가 {df.isnull().sum().sum()}개 있습니다. 해당 행을 제거합니다.")
            df = df.dropna()

        # 1. Robot Position (cm 단위 유지)
        robot_pos = df[['Robot_X', 'Robot_Y', 'Robot_Z']].to_numpy()
        
        # 2. Camera Observation (m -> cm 변환)
        # 사용자의 요청에 따라 m 단위 데이터를 cm로 변환하여 로봇 좌표계와 단위를 맞춤
        cam_measure_m = df[['Cam_X', 'Cam_Y', 'Cam_Z']].to_numpy()
        cam_measure_cm = cam_measure_m * 100.0 
        
        # 3. Robot Rotation Matrix (B_R_E) 생성
        thetas = df['B_R_E'].to_numpy()
        
        zeros = np.zeros_like(thetas)
        ones = np.ones_like(thetas)
        c = np.cos(thetas)
        s = np.sin(thetas)
        
        robot_rot_matrices = np.array([
            [c, -s, zeros],
            [s,  c, zeros],
            [zeros, zeros, ones]
        ]).transpose(2, 0, 1)
        
        return robot_pos, robot_rot_matrices, cam_measure_cm

    def _error_function(self, params, robot_pos, robot_rot, cam_measure):
        # params: Rotation Vector
        target_R_obj = R.from_rotvec(params)
        target_R = target_R_obj.as_matrix()
        
        # 1. R_target * P_cam (이미 cm 단위)
        rotated_cam = np.einsum('ij,nj->ni', target_R, cam_measure)
        
        # 2. t_offset 더하기 (cm 단위)
        offset_added = self.offset_t + rotated_cam
        
        # 3. R_robot * (...) (로봇 회전 적용)
        term_transformed = np.einsum('nij,nj->ni', robot_rot, offset_added)
        
        # 4. 최종 월드 좌표 추정
        p_world_estimates = robot_pos + term_transformed
        
        # 분산(Variance) 최소화 관점: 평균 위치와의 편차 계산
        center = np.mean(p_world_estimates, axis=0)
        residuals = p_world_estimates - center
        return residuals.flatten()

    def solve(self, csv_path):
        try:
            robot_pos, robot_rot, cam_measure = self.load_data(csv_path)
            print(f"[Info] 데이터 로드 완료. (Camera 데이터를 m -> cm로 변환함)")
        except Exception as e:
            print(f"[Critical Error] 데이터 로드 중 치명적 오류 발생: {e}")
            return None

        if len(robot_pos) < 3:
            print(f"[Failure] 데이터 부족: 최소 3개 이상의 데이터 포인트가 필요합니다.")
            return None

        initial_guess = np.array([0.0, 0.0, 0.0])
        
        print(f"최적화 시작 (데이터 {len(robot_pos)}개, Levenberg-Marquardt)...")
        
        result = least_squares(
            self._error_function, 
            initial_guess, 
            args=(robot_pos, robot_rot, cam_measure),
            method='lm'
        )

        final_rvec = result.x
        final_R = R.from_rotvec(final_rvec).as_matrix()
        
        # 결과 분석 (단위: cm)
        residuals = result.fun.reshape(-1, 3)
        residual_norms = np.linalg.norm(residuals, axis=1)
        mean_error_cm = np.mean(residual_norms)
        max_error_cm = np.max(residual_norms)

        print("-" * 50)
        if result.success:
            print(f"[Success] 최적화 수렴 성공")
        else:
            print(f"[Failure] 최적화 수렴 실패")
        print("-" * 50)

        print(f"종료 상태: {result.status} ({result.message})")
        print(f"평균 위치 오차: {mean_error_cm:.4f} cm")
        print(f"최대 위치 오차: {max_error_cm:.4f} cm")

        # 실패 원인 상세 분석
        if not result.success:
            print("\n[상세 분석]")
            if result.status == 0:
                print(" -> 원인: 최대 반복 횟수 초과. 데이터 노이즈 과다 또는 초기값 문제.")
            elif result.status == -1:
                print(" -> 원인: 데이터에 NaN/Inf 포함됨.")
            return None

        THRESHOLD_ERROR_CM = 1.0 
        
        if mean_error_cm > THRESHOLD_ERROR_CM:
            print(f"\n[Warning] 수렴했으나 평균 오차가 큽니다 ({mean_error_cm:.2f} cm).")
            print(" -> 체크리스트 1: Offset 벡터가 cm 단위로 정확한지 확인")
            print(" -> 체크리스트 2: 로봇/카메라 데이터의 타임스탬프 싱크 확인")
            print(" -> 체크리스트 3: 로봇 회전각(B_R_E)이 Radian인지 Degree인지 확인 (코드는 Radian 가정)")
        
        print("\n[Result Matrix E_R_C]")
        with np.printoptions(precision=4, suppress=True):
            print(final_R)
            
        euler = R.from_rotvec(final_rvec).as_euler('xyz', degrees=True)
        print(f"Euler Angles (deg): {euler}")
            
        return final_R
    
if __name__ == "__main__":
    # 1. 이미 알고 있는 Translation Offset (x, y, z) 입력
    # 예: 엔드이펙터에서 카메라까지 x축으로 5cm 떨어져 있다면 [0.05, 0, 0]
    known_offset = [-5.7, 2.9,-21.51] 

    # 2. 클래스 인스턴스 생성
    calibrator = HandEyeCalibration(offset_vector=known_offset)
    
    result_matrix = calibrator.solve("handeye_data_20251212_230734.csv")
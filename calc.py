import numpy as np

def get_rotation_matrix(theta):
    """2D 회전 행렬 반환 (SO2)"""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])

# --- 데이터 정의 ---
# 최종 결과값 (LHS)
final_pos = np.array([-66.2, -12.8])

# 1. Outer Translation
t_outer = np.array([-73.25, 2.4])

# 2. Outer Rotation (0.084 rad)
theta_outer = 0.2946

# 3. Reflection Matrix
m_flip = np.array([[-1, 0], [0, 1]])

# 4. Inner Translation
t_inner = np.array([-5.7, 3.2])

# 5. Source Vector (변환 전 벡터)
v_source = np.array([-12.02, 15.2])

# --- 5단계 계산 수행 ---

# Step 1: Move translation vector
# [-66.2, -12.8] - [-73.19, 3.76]
v1 = final_pos - t_outer
print(f"Step 1 Result: {v1}")

# Step 2: Remove the outer rotation SO2(0.084)
# 역행렬(Inverse)은 -0.084 회전과 같습니다.
m_flip_inv = np.linalg.inv(m_flip) 
R_outer_inv = get_rotation_matrix(-theta_outer)
v2 =  m_flip_inv@ v1
print(f"Step 2 Result: {v2}")

# Step 3: Remove the reflection matrix
# [-1, 0; 0, 1]의 역행렬은 자기 자신입니다.

v3 =  R_outer_inv@ v2
print(f"Step 3 Result: {v3}")

# Step 4: Subtract the inner translation
v4 = v3 - t_inner
print(f"Step 4 Result: {v4}")

# Step 5: Solve for x
# SO2(x) @ v_source ≈ v4
# 따라서 x = Angle(v4) - Angle(v_source)
angle_target = np.arctan2(v4[1], v4[0])
angle_source = np.arctan2(v_source[1], v_source[0])

x = angle_target - angle_source

# 각도를 -pi ~ pi 사이로 정규화 (선택사항, 보통 회전값은 이 범위로 표현)
x = (x + np.pi) % (2 * np.pi) - np.pi

print("-" * 30)
print(f"Final Result x (rad): {x:.4f}")
print(f"Final Result x (deg): {np.degrees(x):.4f}")
print(get_rotation_matrix(x))
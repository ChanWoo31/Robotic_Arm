import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ————— 로봇 파라미터 (m) —————
L1      = 0.140   # 링크1 길이 (어깨→팔꿈치)
L2      = 0.140   # 링크2 길이 (팔꿈치→툴)
Z_OFF   = 0.080   # 베이스에서 어깨축 높이

# ————— 시간 및 관절 궤적 정의 —————
t       = np.arange(0, 10.1, 0.1)           # 0 ~ 10, step=0.1
# deg 단위로 식 정의 후 rad 변환
theta1  = np.deg2rad(9   * t)               # 베이스 yaw
theta2  = np.deg2rad(2   * t)               # 어깨 pitch
theta3  = np.deg2rad(-t**2 + 3.5 * t)       # 팔꿈치 pitch

# ————— 고정 관절 위치 —————
J0 = np.array([0.0, 0.0,      0.0     ])  # 베이스
J1 = np.array([0.0, 0.0,      Z_OFF   ])  # 어깨

# ————— 각 스텝별 관절 위치 계산 —————
pts = np.zeros((len(t), 4, 3))  # shape=(time, joint_index 0~3, xyz)

for i in range(len(t)):
    t1, t2, t3 = theta1[i], theta2[i], theta3[i]

    # 어깨에서 팔꿈치(관절2) 위치
    J2 = J1 + np.array([
        L1 * np.cos(t2) * np.cos(t1),
        L1 * np.cos(t2) * np.sin(t1),
        L1 * np.sin(t2)
    ])
    # 팔꿈치에서 엔드이펙터(관절3) 위치
    J3 = J1 + np.array([
        (L1 * np.cos(t2) + L2 * np.cos(t2 + t3)) * np.cos(t1),
        (L1 * np.cos(t2) + L2 * np.cos(t2 + t3)) * np.sin(t1),
        L1 * np.sin(t2) + L2 * np.sin(t2 + t3)
    ])

    pts[i,0,:] = J0
    pts[i,1,:] = J1
    pts[i,2,:] = J2
    pts[i,3,:] = J3

# ————— 3D 플롯 —————
fig = plt.figure(figsize=(8,6))
ax  = fig.add_subplot(111, projection='3d')
ax.set_title('3-DOF Robot Arm Trajectory')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.grid(True)

# 10스텝마다 궤적 + 링크 그리기
for i in range(0, len(t), 10):
    # 각 관절을 잇는 선
    X = pts[i,:,0]
    Y = pts[i,:,1]
    Z = pts[i,:,2]
    ax.plot(X, Y, Z, '-o', alpha=0.6)

plt.show()

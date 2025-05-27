import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# — Robot Geometry (m) —
L1, L2, Z_OFFSET = 0.140, 0.140, 0.080  # 링크1, 링크2 길이 및 어깨 축 높이

# — Trajectory Parameters —
cx, cy, cz = 0.10, 0.00, 0.00  # 원 중심 (m), cz=0이면 바닥에 닿는 위치
radius = 0.05                 # 원 반지름 (m)
steps = 120                   # 분할 개수

# — Inverse Kinematics (rad) —
def ikine(x, y, z):
    t1 = np.arctan2(y, x)
    r  = np.hypot(x, y)
    s  = z - Z_OFFSET
    D  = (r**2 + s**2 - L1**2 - L2**2) / (2 * L1 * L2)
    D  = np.clip(D, -1.0, 1.0)
    t3 = np.arccos(D)
    alpha = np.arctan2(s, r)
    beta  = np.arctan2(L2 * np.sin(t3), L1 + L2 * np.cos(t3))
    t2 = alpha - beta
    return t1, t2, t3

# — Compute full trajectory of joints and end-effector —
angles = np.linspace(0, 2*np.pi, steps+1)

# Base (J0) and shoulder (J1) fixed positions
J0 = np.array([0., 0., 0.])
J1 = np.array([0., 0., Z_OFFSET])

traj_J2 = []  # 팔꿈치 관절 위치
traj_J3 = []  # 엔드이펙터 위치

for ang in angles:
    x = cx + radius * np.cos(ang)
    y = cy + radius * np.sin(ang)
    z = cz

    t1, t2, t3 = ikine(x, y, z)

    # Joint2 (elbow) position
    j2 = J1 + np.array([
        L1 * np.cos(t2) * np.cos(t1),
        L1 * np.cos(t2) * np.sin(t1),
        L1 * np.sin(t2)
    ])
    traj_J2.append(j2)

    # End-effector position
    j3 = J1 + np.array([
        (L1 * np.cos(t2) + L2 * np.cos(t2 + t3)) * np.cos(t1),
        (L1 * np.cos(t2) + L2 * np.cos(t2 + t3)) * np.sin(t1),
        L1 * np.sin(t2) + L2 * np.sin(t2 + t3)
    ])
    traj_J3.append(j3)

traj_J2 = np.array(traj_J2)
traj_J3 = np.array(traj_J3)

# — Plot 3D Trajectories —
fig = plt.figure(figsize=(8,6))
ax = fig.add_subplot(111, projection='3d')

# End-effector path
ax.plot(traj_J3[:,0], traj_J3[:,1], traj_J3[:,2],
        color='orange', linewidth=2, label='End-Effector')

# Elbow path
ax.plot(traj_J2[:,0], traj_J2[:,1], traj_J2[:,2],
        color='blue', linewidth=1.5, label='Elbow Joint')

# 몇 가지 대표 자세(0, 1/4, 1/2, 3/4)
for idx in [0, steps//4, steps//2, 3*steps//4]:
    pts = np.vstack((J0, J1, traj_J2[idx], traj_J3[idx]))
    ax.plot(pts[:,0], pts[:,1], pts[:,2],
            color='gray', alpha=0.4)

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('3D Robot Arm Trajectory')
ax.legend()
ax.set_box_aspect([1,1,0.5])
plt.show()

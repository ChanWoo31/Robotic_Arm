import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from ikpy.link import OriginLink, DHLink
from ikpy.chain import Chain

# ─── 1) 4-DOF DH 체인 정의 ─────────────────────────────────────
d1 = 110.0
a2, a3, a4 = 140.0, 140.0, 80.0

robot_chain = Chain(name='4dof_pen_mount', links=[
    OriginLink(),
    # Joint1: 베이스 Yaw
    DHLink(d=d1, a=0,   alpha=np.deg2rad(90), theta=0),
    # Joint2: 어깨 Pitch
    DHLink(d=0,  a=a2,  alpha=0,               theta=np.deg2rad(90)),
    # Joint3: 팔 Pitch
    DHLink(d=0,  a=a3,  alpha=0,               theta=0),
    # Joint4: 펜 마운트 (툴링) 회전축을 제어
    DHLink(d=0,  a=a4,  alpha=0,               theta=0),
])

# ─── 2) 원 궤적 파라미터 ───────────────────────────────────────
x0, y0, z0, r0 = 160, 0, 100, 50   # 원의 중심 (mm) & 반지름
steps = 100                        # 분할 개수

# ─── 3) IK → FK → 프레임별 링크 위치 저장 ────────────────────
frames = []
for i in range(steps):
    θ = 2 * np.pi * i / steps
    target = [x0 + r0*np.cos(θ),
              y0 + r0*np.sin(θ),
              z0]

    # 4-DOF IK: 위치 + 마지막 링크(local Y축)이 전역 Z축과 정렬 → 링크4은 항상 수평
    thetas = robot_chain.inverse_kinematics(
        target_position=target,
        orientation_mode='Y',
        target_orientation=[0, 0, 1]
    )

    # 모든 링크의 동차 변환 행렬 획득
    transforms = robot_chain.forward_kinematics(thetas, full_kinematics=True)
    pts = [T[:3, 3].copy() for T in transforms]  # [Base, J1, J2, J3, J4]

    frames.append(pts)

# ─── 4) Matplotlib 3D 애니메이션 ───────────────────────────────
fig = plt.figure(figsize=(10, 8))
ax  = fig.add_subplot(projection='3d')
ax.set_box_aspect((1,1,1))
ax.view_init(elev=30, azim=-60)

ax.set_xlim(x0-r0-20, x0+r0+20)
ax.set_ylim(y0-r0-20, y0+r0+20)
ax.set_zlim(z0-20, z0+20)
ax.set_xlabel('X (mm)')
ax.set_ylabel('Y (mm)')
ax.set_zlabel('Z (mm)')

# 링크 4개 선 객체
colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:purple']
lines  = [ax.plot([], [], [], lw=4, color=colors[i], marker='o')[0]
          for i in range(4)]

# end-effector(펜 끝)의 궤적 (빨간 점선)
trace, = ax.plot([], [], [], '--', lw=2, color='tab:red', alpha=0.7)
tx, ty, tz = [], [], []

def init():
    for ln in lines + [trace]:
        ln.set_data([], [])
        ln.set_3d_properties([])
    return lines + [trace]

def update(frame):
    pts = frames[frame]
    # 각 링크 그리기
    for i, ln in enumerate(lines):
        p0, p1 = pts[i], pts[i+1]
        ln.set_data([p0[0], p1[0]], [p0[1], p1[1]])
        ln.set_3d_properties([p0[2], p1[2]])
    # 궤적 업데이트 (펜 끝 = pts[-1])
    ee = pts[-1]
    tx.append(ee[0]); ty.append(ee[1]); tz.append(ee[2])
    trace.set_data(tx, ty)
    trace.set_3d_properties(tz)
    return lines + [trace]

ani = FuncAnimation(fig, update, frames=steps,
                    init_func=init, blit=True, interval=50)
plt.show()

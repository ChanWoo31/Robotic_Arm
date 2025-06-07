import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from ikpy.link import OriginLink, DHLink
from ikpy.chain import Chain

# ─── 1) 로봇 DH 파라미터 체인 정의 ───────────────────────────
d1 = 110.0
a2, a3, a4 = 140.0, 140.0, 80.0

robot_chain = Chain(name='4dof_dxl', links=[
    OriginLink(),
    DHLink(d=d1, a=0,   alpha=np.deg2rad(90), theta=0),
    DHLink(d=0,  a=a2,  alpha=0,               theta=np.deg2rad(90)),
    DHLink(d=0,  a=a3,  alpha=0,               theta=0),
    DHLink(d=0,  a=a4,  alpha=0,               theta=0),
])

# ─── 2) 원 궤적 파라미터 ──────────────────────────────────────
x0, y0, z0, r0 = 160, 0, 100, 50   # 원의 중심과 반지름(mm)
steps = 100                         # 분할 개수

# ─── 3) IK → FK → 링크 위치 및 궤적 계산 ─────────────────────
frames = []
for i in range(steps):
    theta_c = 2*np.pi * i/steps
    px, py, pz = x0 + r0*np.cos(theta_c), y0 + r0*np.sin(theta_c), z0

    # 순수 위치 IK (orientation 무시)
    thetas = robot_chain.inverse_kinematics([px, py, pz])
    transforms = robot_chain.forward_kinematics(thetas, full_kinematics=True)
    pts = [T[:3,3].copy() for T in transforms]  # base→J1→J2→J3→J4/EE

    frames.append((pts, np.array([px, py, pz])))

# ─── 4) 애니메이션 셋업 ───────────────────────────────────────
fig = plt.figure(figsize=(10,8))
ax  = fig.add_subplot(projection='3d')
ax.set_box_aspect((1,1,1))
ax.view_init(elev=30, azim=-60)
ax.set_xlim(x0-r0-30, x0+r0+30)
ax.set_ylim(y0-r0-30, y0+r0+30)
ax.set_zlim(z0-a4-30, z0+30)
ax.set_xlabel('X (mm)'); ax.set_ylabel('Y (mm)'); ax.set_zlabel('Z (mm)')

# 실제 3개 링크
kin_lines = [ax.plot([], [], [], lw=4, marker='o')[0] for _ in range(3)]
# 강제 수직 아래 툴 링크 (빨간색)
tool_line = ax.plot([], [], [], 'r-', lw=4)[0]
# 원 궤적(점선)
trace_line = ax.plot([], [], [], '--', lw=2, alpha=0.7)[0]

trace_x, trace_y, trace_z = [], [], []

def init():
    for ln in kin_lines + [tool_line, trace_line]:
        ln.set_data([], []); ln.set_3d_properties([])
    return kin_lines + [tool_line, trace_line]

def update(frame):
    pts, ee = frames[frame]
    # 1) 첫 3개 링크
    for i, ln in enumerate(kin_lines):
        p0, p1 = pts[i], pts[i+1]
        ln.set_data([p0[0], p1[0]], [p0[1], p1[1]])
        ln.set_3d_properties([p0[2], p1[2]])
    # 2) 빨간툴: J4 위치(pts[3])에서 전역 –Z 방향으로 a4만큼
    j4 = pts[3]
    te = j4 + np.array([0, 0, -a4])
    tool_line.set_data([j4[0], te[0]], [j4[1], te[1]])
    tool_line.set_3d_properties([j4[2], te[2]])
    # 3) 원 궤적 갱신
    trace_x.append(ee[0]); trace_y.append(ee[1]); trace_z.append(ee[2])
    trace_line.set_data(trace_x, trace_y)
    trace_line.set_3d_properties(trace_z)

    return kin_lines + [tool_line, trace_line]

ani = FuncAnimation(fig, update, frames=steps,
                    init_func=init, blit=True, interval=50)
plt.show()

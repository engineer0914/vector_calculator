from functions_sim_for_850 import Transform3D, RobotArm, Camera
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

print("--- [Eye-in-Hand] 로봇-그리퍼-객체 접근 시뮬레이션 ---")

# =========================================================
# 1. 로봇 및 도구(그리퍼, 카메라) 설정
# =========================================================

# 1-1. 로봇 생성
try:
    # 앞서 생성한 RB3-730 DH 파라미터 파일 사용
    robot = RobotArm(num_axes=6, dh_param_file='rb3_730_dh.csv')
except FileNotFoundError as e:
    print(e)
    exit()

# 1-2. 로봇 자세 설정 (Home 포즈 등)
# 카메라가 물체를 잘 바라보도록 4, 5번 관절을 좀 굽혀봅니다.
joint_angles = [0, 0, -90, -90, 0, 0] 
robot.set_joint_angles(joint_angles)

# 1-3. 현재 엔드 이펙터(EE) 포즈 계산 (T_base_ee)
T_base_ee = robot.get_end_effector_pose()
print("\n[1] 로봇 EE 포즈 (Base 기준):\n", T_base_ee)


# =========================================================
# 2. 오프셋 설정 (Eye-in-Hand & Gripper)
# =========================================================

# 2-1. 카메라 오프셋 (엔드이펙터 -> 카메라)
# "엔드이펙터 좌표계에서 x축 50mm, z축 50mm 이동"
# 회전은 없다고 가정 (필요시 rx, ry, rz 추가)
T_ee_cam = Transform3D.from_xyz_rpy(x=50.0, y=0.0, z=50.0, rx=0, ry=0, rz=0)

# 로봇 베이스 기준 카메라 포즈 계산 (T_base_cam = T_base_ee @ T_ee_cam)
T_base_cam = T_base_ee @ T_ee_cam
print("\n[2] 카메라 포즈 (Base 기준):\n", T_base_cam)

# 카메라 객체 생성 (좌표 변환용)
camera = Camera(T_base_cam)


# 2-2. 그리퍼(TCP) 오프셋 (엔드이펙터 -> 그리퍼 끝)
# "엔드이펙터에서 z축 방향으로 100mm"
T_ee_gripper = Transform3D.from_xyz_rpy(x=0.0, y=0.0, z=100.0, rx=0, ry=0, rz=0)

# 로봇 베이스 기준 그리퍼 포즈 계산 (T_base_gripper = T_base_ee @ T_ee_gripper)
T_base_gripper = T_base_ee @ T_ee_gripper
print("\n[3] 그리퍼(TCP) 포즈 (Base 기준):\n", T_base_gripper)


# =========================================================
# 3. 객체 검출 및 접근점 계산
# =========================================================

# 3-1. 카메라가 감지한 객체 포즈 (T_cam_object)
# 가정: 카메라 앞쪽(Z축) 300mm, 약간 오른쪽(X 50mm)에 물체가 있고, Y축으로 10도 회전되어 있음
T_cam_object = Transform3D.from_xyz_rpy(x=50.0, y=0.0, z=300.0, rx=0, ry=90, rz=90, degrees=True)
print("\n[4] 카메라가 본 객체 (Cam 기준):\n", T_cam_object)


# 3-2. 로봇 베이스 기준 객체 포즈 계산 (T_base_object)
# T_base_object = T_base_cam @ T_cam_object
T_base_object = camera.transform_pose_from_camera_to_base_frame(T_cam_object)
print("\n[5] ★ 객체 실제 위치 (Base 기준) ★:\n", T_base_object)


# 3-3. 진입(Approach) 위치 계산
# "객체의 X축을 기준으로 직선으로 200mm 만큼 오프셋"
# (진입하려면 보통 객체 앞 -200mm 지점이 안전하므로 -200으로 설정, 필요시 +200 수정)
approach_distance = 200.0 
T_obj_approach_offset = Transform3D.from_xyz_rpy(x=approach_distance, y=0, z=0, rx=0, ry=0, rz=0)

# 베이스 기준 진입점 포즈 = 객체포즈 @ 진입오프셋
T_base_approach = T_base_object @ T_obj_approach_offset
print(f"\n[6] 진입 대기 위치 (Base 기준, 객체 X축 {approach_distance}mm):\n", T_base_approach)


# =========================================================
# 4. 3D 시각화
# =========================================================

def visualize_scene():
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title(f'RB3-730 Eye-in-Hand Simulation\nObject Approach (Offset {approach_distance}mm)')
    ax.set_xlabel('X [mm]')
    ax.set_ylabel('Y [mm]')
    ax.set_zlabel('Z [mm]')
    
    # 뷰 범위 설정
    limit = 800
    ax.set_xlim([-limit, limit])
    ax.set_ylim([-limit, limit])
    ax.set_zlim([0, limit*1.5])
    ax.view_init(elev=30, azim=45)

    # --- 헬퍼 함수: 좌표계 그리기 ---
    def draw_frame(T, label, scale=50, lw=2):
        o, x, y, z = T.get_axes_vectors(scale)
        ax.scatter(*o, color='k', s=20)
        ax.quiver(*o, *(x-o), color='r', lw=lw) # X
        ax.quiver(*o, *(y-o), color='g', lw=lw) # Y
        ax.quiver(*o, *(z-o), color='b', lw=lw) # Z
        if label:
            ax.text(o[0], o[1], o[2], label, fontsize=9)

    # 1. 로봇 링크 그리기
    link_poses = robot.get_all_link_poses()
    points = [np.zeros(3)] + [p.get_origin() for p in link_poses]
    
    # 링크 선
    xs, ys, zs = zip(*points)
    ax.plot(xs, ys, zs, 'o-', color='gray', lw=3, label='Robot Arm')
    
    # 각 관절 좌표계
    for i, pose in enumerate(link_poses[:-1]):
        draw_frame(pose, f'J{i+1}', scale=30, lw=1)
    
    # 2. 엔드 이펙터 (EE) 좌표계
    draw_frame(T_base_ee, 'EE', scale=60, lw=2)

    # 3. 카메라 좌표계 (EE에 부착됨)
    draw_frame(T_base_cam, 'Camera', scale=60, lw=2)
    # 카메라 위치 점
    cx, cy, cz = T_base_cam.get_origin()
    ax.scatter(cx, cy, cz, color='purple', s=100, marker='^', label='Eye-in-Hand Cam')
    
    # 4. 그리퍼(TCP) 좌표계 (EE에 부착됨)
    draw_frame(T_base_gripper, 'Gripper(TCP)', scale=60, lw=2)
    gx, gy, gz = T_base_gripper.get_origin()
    ax.scatter(gx, gy, gz, color='cyan', s=100, marker='v', label='Gripper Tip')
    
    # EE -> Camera / EE -> Gripper 연결선 (구조적 연결 표시)
    ex, ey, ez = T_base_ee.get_origin()
    ax.plot([ex, cx], [ey, cy], [ez, cz], '--', color='purple', lw=1)
    ax.plot([ex, gx], [ey, gy], [ez, gz], '--', color='cyan', lw=1)

    # 5. 객체 (Object) 좌표계
    draw_frame(T_base_object, 'Object', scale=80, lw=2)
    ox, oy, oz = T_base_object.get_origin()
    ax.scatter(ox, oy, oz, color='orange', s=150, marker='s', label='Detected Object')

    # 카메라 시선 (Cam -> Obj)
    ax.plot([cx, ox], [cy, oy], [cz, oz], ':', color='green', label='Cam Sight')

# 6. 진입점 (Approach) 좌표계
    draw_frame(T_base_approach, 'Approach', scale=60, lw=2)
    
    # 변수명을 ax와 겹치지 않게 수정 (ap_x, ap_y, ap_z)
    ap_x, ap_y, ap_z = T_base_approach.get_origin() 
    
    # ax.scatter를 호출할 때 수정된 좌표 변수 사용
    ax.scatter(ap_x, ap_y, ap_z, color='red', s=100, marker='x', label='Approach Point')

    # 진입 경로 (Approach -> Object) 그리기 부분도 수정된 변수 사용
    # fig.axes[0] 대신 그냥 ax를 써도 되지만, 안전하게 수정된 좌표 변수 사용
    ax.plot([ap_x, ox], [ap_y, oy], [ap_z, oz], '--', color='red', lw=2, label='Approach Path')

    fig.legend(loc='upper right')
    plt.show()

# T_base_approach 변수에서 6D 정보 추출하기

# 1. 위치 (Position: X, Y, Z)
# get_translation() 또는 get_origin() 사용
approach_pos = T_base_approach.get_translation()
x = approach_pos[0]
y = approach_pos[1]
z = approach_pos[2]

# 2. 회전 (Orientation: RX, RY, RZ)
# get_euler_angles() 사용 (degrees=True로 설정하여 도 단위 변환)
approach_rot = T_base_approach.get_euler_angles(degrees=True)
rx = approach_rot[0]
ry = approach_rot[1]
rz = approach_rot[2]

print("-" * 40)
print("🚀 [진입점(Approach Point) 6D 좌표 분석]")
print(f"변수명: T_base_approach")
print(f"----------------------------------------")
print(f"X  : {x:.4f} mm")
print(f"Y  : {y:.4f} mm")
print(f"Z  : {z:.4f} mm")
print(f"RX : {rx:.4f} deg")
print(f"RY : {ry:.4f} deg")
print(f"RZ : {rz:.4f} deg")
print("-" * 40)

visualize_scene()
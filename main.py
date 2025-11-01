# main.py

from functions import Transform3D, RobotArm, Camera
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# --- 0. 사전 준비 ---
# [!] 중요: 이 스크립트를 실행하기 전에,
# 터미널에서 create_dh_csv.py 를 먼저 실행해서 
# 'rb5_850_dh.csv' 파일이 같은 폴더에 있는지 확인해야 합니다.
# ( $ python create_dh_csv.py )
# --------------------


print("--- 로봇-카메라 좌표계 변환 시뮬레이션 (DH 파라미터 적용) ---")

# --- 1. 카메라 설정 (보정) ---
print("\n[1] 카메라 보정 행렬(T_base_cam) 설정...")
# T_base_cam: 로봇 베이스 기준 카메라의 포즈
T_base_cam = Transform3D.from_xyz_rpy(x=1000.0, y=0.0, z=500.0, # 단위를 mm로 조정
                                      rx=0.0, ry=-90.0, rz=0.0, 
                                      degrees=True)
camera = Camera(T_base_cam) # 수정된 부분: T_base_to_cam -> T_base_cam
print(f"카메라 보정 (T_base_cam):\n{camera.T_base_cam}")


# --- 2. 로봇팔 생성 및 조작 (DH 파라미터 파일 지정) ---
print("\n[2] 로봇팔 생성 및 관절 이동...")
try:
    # [수정됨] DH 파라미터 파일을 지정하여 로봇 객체 생성
    # RB5-850 DH 파라미터가 mm 단위이므로, 시각화도 mm 단위로 가정
    robot = RobotArm(num_axes=6, dh_param_file='rb5_850_dh.csv')
except FileNotFoundError as e:
    print(e)
    print("스크립트를 종료합니다. create_dh_csv.py를 먼저 실행해 주세요.")
    exit()

# 로봇 관절 각도를 'Home' 자세 (예시)로 설정 (단위: 도)
joint_angles = [0, 0, -90, 0, -90, 0] # 관절 각도 변경 가능
robot.set_joint_angles(joint_angles)


# --- 3. 로봇 엔드 이펙터(EE) 포즈 계산 (실제 FK) ---
print("\n[3] 로봇 EE 포즈 계산 (Base 기준)...")
T_base_ee = robot.get_end_effector_pose()

print("--- 로봇 EE 포즈 (T_base_ee) ---")
print(T_base_ee)


# --- 4. (Goal 1) 검출된 객체 포즈 변환 시뮬레이션 ---
print("\n[4] 카메라가 검출한 객체 포즈 변환...")
# 가상: 카메라가 자신의 좌표계 기준으로 (x=100, z=800mm) 앞에 있고,
# y축으로 10도 기울어진 물체를 검출했다고 가정합니다.
T_cam_object = Transform3D.from_xyz_rpy(x=100.0, y=0.0, z=800.0, # 단위를 mm로 조정
                                        rx=0, ry=10, rz=0, degrees=True)

print("--- (카메라 기준) 검출된 객체 포즈 (T_cam_object) ---")
print(T_cam_object)

# 이제 로봇이 이 물체를 집을 수 있도록 '로봇 베이스' 기준으로 변환합니다.
T_base_object = camera.transform_pose_from_camera_to_base_frame(T_cam_object)

print("\n--- (로봇 베이스 기준) 변환된 객체 포즈 (T_base_object) ---")
print(T_base_object)
print("-> 로봇은 이 (x,y,z)와 (rx,ry,rz) 값을 타겟으로 역기구학(IK)을 계산해야 합니다.")


print("\n--- 5. 3D 시각화 ---")

# 5.1. 3D 그래프 설정
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X [mm]')
ax.set_ylabel('Y [mm]')
ax.set_zlabel('Z [mm]')
ax.set_title('Robot-Camera-Object in 3D Space')

# [수정] ax.set_aspect('auto') 대신 축 범위를 기반으로 비율을 설정해 봅니다.
# 이는 왜곡을 줄여줄 수 있습니다.
plot_range = 1000 # ±1000mm 범위
ax.set_xlim([-plot_range, plot_range])
ax.set_ylim([-plot_range, plot_range])
ax.set_zlim([0, plot_range * 1.5]) 

# [!!! 핵심 추가 !!!]
# 3D 뷰의 초기 각도를 설정합니다. (Z축이 위로 가도록)
# elev: 뷰의 '높낮이' (고도). 90=위에서 바로 아래, 0=옆에서. (25~30도가 적당)
# azim: 뷰의 '좌우 회전' (방위각). 
ax.view_init(elev=25, azim=-60)

# 5.2. 좌표계 그리기 헬퍼 함수
def draw_frame(ax, T: Transform3D, label="", scale=100):
    """주어진 Transform3D 객체의 원점과 X,Y,Z 축을 그립니다."""
    origin, x_axis_vec, y_axis_vec, z_axis_vec = T.get_axes_vectors(scale=scale)

    # 원점 그리기
    ax.scatter(origin[0], origin[1], origin[2], marker='o', s=50, color='black')
    
    # 축 벡터 그리기 (RGB = X, Y, Z)
    ax.quiver(origin[0], origin[1], origin[2],
              x_axis_vec[0] - origin[0], x_axis_vec[1] - origin[1], x_axis_vec[2] - origin[2],
              color='red', linewidth=2, arrow_length_ratio=0.1) # X-axis (Red)
    ax.quiver(origin[0], origin[1], origin[2],
              y_axis_vec[0] - origin[0], y_axis_vec[1] - origin[1], y_axis_vec[2] - origin[2],
              color='green', linewidth=2, arrow_length_ratio=0.1) # Y-axis (Green)
    ax.quiver(origin[0], origin[1], origin[2],
              z_axis_vec[0] - origin[0], z_axis_vec[1] - origin[1], z_axis_vec[2] - origin[2],
              color='blue', linewidth=2, arrow_length_ratio=0.1) # Z-axis (Blue)

    # 라벨 추가
    ax.text(origin[0] + scale * 0.1, origin[1] + scale * 0.1, origin[2], label, color='black')


# 5.3. 로봇 베이스 좌표계 그리기 (World Frame과 동일하다고 가정)
draw_frame(ax, Transform3D.identity(), label='World/Base Frame', scale=150)


# 5.4. 로봇팔 링크 그리기
print("로봇팔 링크와 조인트를 그립니다...")
link_poses = robot.get_all_link_poses()
joint_points = [Transform3D.identity().get_origin()] # 베이스 원점을 첫 조인트로 시작
for i, T_link in enumerate(link_poses):
    joint_points.append(T_link.get_origin())
    draw_frame(ax, T_link, label=f'Link {i+1} End', scale=50) # 각 링크 끝의 좌표계

# 조인트들을 선으로 연결하여 로봇팔 구조를 시각화 (각 조인트는 이전 조인트의 끝)
for i in range(len(joint_points) - 1):
    p1 = joint_points[i]
    p2 = joint_points[i+1]
    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 'k-', linewidth=3, alpha=0.7)


# 5.5. 카메라 좌표계 그리기
print("카메라 좌표계를 그립니다...")
draw_frame(ax, camera.T_base_cam, label='Camera Frame', scale=100)
# 카메라 위치를 나타내는 카메라 아이콘이나 단순한 점 추가 (선택 사항)
cam_origin = camera.T_base_cam.get_origin()
ax.scatter(cam_origin[0], cam_origin[1], cam_origin[2], 
           marker='^', s=200, color='purple', label='Camera Position')


# 5.6. 검출된 객체 좌표계 그리기 (로봇 베이스 기준)
print("검출된 객체 좌표계(베이스 기준)를 그립니다...")
draw_frame(ax, T_base_object, label='Detected Object (Base)', scale=80)
# 객체 중심점을 나타내는 아이콘 추가
obj_origin = T_base_object.get_origin()
ax.scatter(obj_origin[0], obj_origin[1], obj_origin[2], 
           marker='s', s=150, color='orange', label='Object Position')


# 범례 표시
ax.legend()

# 3D 그래프 보여주기
plt.tight_layout()
plt.show()

print("\n--- 시뮬레이션 종료 ---")
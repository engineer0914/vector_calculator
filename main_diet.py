# main.py

from functions_sim_for_850 import Transform3D, RobotArm, Camera
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


print("--- 로봇-카메라 DH 파라미터 적용 좌표계 변환 시뮬레이션 ---")

# --- 1. 카메라 설정 (보정) ---
# T_base_cam: 로봇 베이스 기준 카메라의 포즈
# 로봇의 베이스를 0,0이라 할때 카메라가 위치 및 회전 상태를 보여줌

print("\n[1] 카메라의 6D 지정후 4*4 변환")

# 6D -> 4*4 행렬로 변환


ay = -285.0
T_base_cam = Transform3D.from_xyz_rpy(x=-115.0, y = ay, z=790.0,
                                      rx=0.0, ry=180.0, rz=0.0,
                                      degrees=True)
# print(T_base_cam)

camera = Camera(T_base_cam)

# print(f"베이스 -> 카메라 (T_base_cam):\n{camera.T_base_cam}")
# print(f"카메라 -> 베이스 (T_cam_base):\n{camera.T_cam_base}")








# --- 2. 로봇팔 생성 및 조작 (DH 파라미터 파일 지정) ---
print("\n[2] 로봇팔 생성 및 관절 이동...")

# 로봇팔 DH 파라미터 가져오기
try:
    robot = RobotArm(num_axes=6, dh_param_file='rb5_850_dh.csv')
except FileNotFoundError as e:
    print(e)
    print("스크립트를 종료합니다. create_dh_csv.py를 먼저 실행해 주세요.")
    exit()

# 로봇 관절 각도를 'Home' 자세 (예시)로 설정 (단위: 도)
joint_angles = [0, 0, 0, 0, 0, 0] # 관절 각도 변경 가능
robot.set_joint_angles(joint_angles)







# --- 3. 로봇 엔드 이펙터(EE) 포즈 계산 (실제 FK) ---
print("\n[3] 로봇 EE 포즈 계산 (Base 기준)...")
T_base_ee = robot.get_end_effector_pose()

print("--- 로봇 EE 포즈 (T_base_ee) ---")
print(T_base_ee)







# --- 4. (Goal 1) 검출된 객체 포즈 변환 시뮬레이션 ---
print("\n[4] 카메라가 검출한 객체 포즈 변환...")

# 카메라 -> 물체
# x 100, z 800, y 10 deg

T_cam_object = Transform3D.from_xyz_rpy(x=250.0, y=300.0, z=500.0,
                                        rx=0, ry=10, rz=0, degrees=True)

print("--- (카메라 기준) 검출된 객체 포즈 (T_cam_object) ---")
print(T_cam_object)

# 이제 로봇이 이 물체를 집을 수 있도록 '로봇 베이스' 기준으로 변환합니다.
T_base_object = camera.transform_pose_from_camera_to_base_frame(T_cam_object)

print("\n--- 로봇 베이스 -> 객체 포즈 (T_base_object) ---")
print(T_base_object)







print("\n--- 5. 3D 시각화 ---")

def robot_3d_visualizer(robot):
    # 5.1. 3D 그래프 설정
    fig = plt.figure(figsize=(12, 10)) 
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X [mm]')
    ax.set_ylabel('Y [mm]')
    ax.set_zlabel('Z [mm]')
    ax.set_title('Robot-Camera-Object in 3D Space')

    plot_range = 1000 
    ax.set_xlim([-plot_range, plot_range])
    ax.set_ylim([-plot_range, plot_range])
    ax.set_zlim([0, plot_range * 1.5]) 

    ax.view_init(elev=25, azim=45)


    # 5.2. 좌표계 그리기 헬퍼 함수
    def draw_frame(ax, T: Transform3D, label="", scale=100, linewidth=2): 
        # ... (이 함수는 수정할 필요 없이 그대로 둡니다) ...
        origin, x_axis_vec, y_axis_vec, z_axis_vec = T.get_axes_vectors(scale=scale)

        ax.scatter(origin[0], origin[1], origin[2], marker='o', s=50, color='black')
        
        ax.quiver(origin[0], origin[1], origin[2],
                x_axis_vec[0] - origin[0], x_axis_vec[1] - origin[1], x_axis_vec[2] - origin[2],
                color='red', linewidth=linewidth, arrow_length_ratio=0.1) # X-axis (Red)
        ax.quiver(origin[0], origin[1], origin[2],
                y_axis_vec[0] - origin[0], y_axis_vec[1] - origin[1], y_axis_vec[2] - origin[2],
                color='green', linewidth=linewidth, arrow_length_ratio=0.1) # Y-axis (Green)
        ax.quiver(origin[0], origin[1], origin[2],
                z_axis_vec[0] - origin[0], z_axis_vec[1] - origin[1], z_axis_vec[2] - origin[2],
                color='blue', linewidth=linewidth, arrow_length_ratio=0.1) # Z-axis (Blue)

        if label: 
            ax.text(origin[0] + scale * 0.1, origin[1] + scale * 0.1, origin[2], label, color='black', fontsize=9)

    # 5.3. 로봇 베이스 좌표계 그리기 (World Frame과 동일하다고 가정)
    draw_frame(ax, Transform3D.identity(), label='World/Base Frame', scale=150, linewidth=3)


    # 5.4. 로봇팔 링크 그리기
    print("로봇팔 링크와 조인트를 그립니다...")
    link_poses = robot.get_all_link_poses()
    joint_points = [Transform3D.identity().get_origin()] # 베이스 원점을 첫 조인트로 시작

    for i, T_link in enumerate(link_poses):
        joint_points.append(T_link.get_origin())
        
        # 엔드 이펙터(마지막 링크의 끝)는 다른 색상으로
        if i == len(link_poses) - 1: # 마지막 링크인 경우
            draw_frame(ax, T_link, label=f'EE Frame', scale=70, linewidth=2) # 엔드 이펙터 프레임
            ax.scatter(T_link.get_origin()[0], T_link.get_origin()[1], T_link.get_origin()[2],
                    marker='X', s=200, color='cyan', label='End Effector Position', depthshade=True) # 더 큰 마커
        else:
            draw_frame(ax, T_link, label=f'Link {i+1} End', scale=50, linewidth=1.5) # 일반 링크 끝의 좌표계

    # 조인트들을 선으로 연결하여 로봇팔 구조를 시각화 (각 조인트는 이전 조인트의 끝)
    for i in range(len(joint_points) - 1):
        p1 = joint_points[i]
        p2 = joint_points[i+1]
        
        # 엔드 이펙터 연결 선도 다른 색상으로
        if i == len(joint_points) - 2: # 마지막 조인트-EE 연결 선
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 'c-', linewidth=4, alpha=0.9, label='EE Link') # Cyan
        else:
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 'k-', linewidth=3, alpha=0.7) # Black


    # 5.5. 카메라 좌표계 그리기
    print("카메라 좌표계를 그립니다...")
    draw_frame(ax, camera.T_base_cam, label='Camera Frame', scale=100, linewidth=2.5)
    cam_origin = camera.T_base_cam.get_origin()
    ax.scatter(cam_origin[0], cam_origin[1], cam_origin[2], 
            marker='^', s=200, color='purple', label='Camera Position', depthshade=True)


    # 카메라 시선 방향 벡터 그리기
    print("카메라 시선 방향을 그립니다...")
    cam_origin_point = camera.T_base_cam.get_origin()
    # 카메라의 Z축 방향이 일반적으로 시선 방향입니다 (Transform3D의 Z축 사용)
    # z_axis_vector는 get_axes_vectors의 4번째 반환값 (scale이 곱해지기 전 단위 벡터)
    cam_rot_mat = camera.T_base_cam.get_rotation_matrix()
    view_direction_unit_vector = cam_rot_mat[:, 2] # Z축 방향 벡터

    view_distance = 150 # 시선 방향을 그릴 거리 (mm)
    target_point = cam_origin_point + view_direction_unit_vector * view_distance

    ax.plot([cam_origin_point[0], target_point[0]], 
            [cam_origin_point[1], target_point[1]], 
            [cam_origin_point[2], target_point[2]], 
            '--o', color='gray', linewidth=1.5, markersize=5, label='Camera View Direction') # 점선으로 표시


    # 5.6. 검출된 객체 좌표계 그리기 (로봇 베이스 기준)
    print("검출된 객체 좌표계(베이스 기준)를 그립니다...")
    draw_frame(ax, T_base_object, label='Detected Object (Base)', scale=80, linewidth=2)
    obj_origin = T_base_object.get_origin()
    ax.scatter(obj_origin[0], obj_origin[1], obj_origin[2], 
            marker='s', s=150, color='orange', label='Object Position', depthshade=True)


    # 5.7 카메라 - 객체 간 점선 그리기
    cam_origin_point = camera.T_base_cam.get_origin()

    target_point = T_base_object.get_origin()

    ax.plot([cam_origin_point[0], target_point[0]], 
            [cam_origin_point[1], target_point[1]], 
            [cam_origin_point[2], target_point[2]], 
            '--o', color='green', linewidth=1.5, markersize=5, label='Cam - Obj')



    # 범례 표시
    ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1.0), fontsize=10) # 범례를 그래프 밖에 표시

    # 축 색상 키(표) 텍스트 상자 추가
    key_text = "Axis Color Key (표)\n" \
            "----------------------\n" \
            "  Red   = X-axis\n" \
            "  Green = Y-axis\n" \
            "  Blue  = Z-axis"



    # 범례(legend) 아래쪽에 텍스트 상자를 위치시킵니다.
    ax.text2D(1.05, 0.75, key_text, transform=ax.transAxes, 
            fontsize=10, verticalalignment='top', 
            bbox=dict(facecolor='white', alpha=0.9, boxstyle='round,pad=0.5'))

    fig.tight_layout(rect=[0, 0, 0.8, 1]) 

    # 3D 그래프 보여주기
    plt.show()

robot_3d_visualizer(robot)

print("\n--- 시뮬레이션 종료 ---")



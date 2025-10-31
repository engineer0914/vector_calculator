# main.py

from functions import Transform3D, RobotArm, Camera
import numpy as np

# --- 0. 사전 준비 ---
# [!] 중요: 이 스크립트를 실행하기 전에,
# 터미널에서 create_dh_csv.py 를 먼저 실행해서 
# 'rb5_850_dh.csv' 파일이 같은 폴더에 있는지 확인해야 합니다.
# ( $ python create_dh_csv.py )
# --------------------


print("--- 로봇-카메라 좌표계 변환 시뮬레이션 (DH 파라미터 적용) ---")

# --- 1. 카메라 설정 (보정) ---
print("\n[1] 카메라 보정 행렬(T_base_cam) 설정...")
T_base_cam = Transform3D.from_xyz_rpy(x=1.0, y=0.0, z=0.5, 
                                      rx=0.0, ry=-90.0, rz=0.0, 
                                      degrees=True)
camera = Camera(T_base_to_cam)


# --- 2. 로봇팔 생성 및 조작 (DH 파라미터 파일 지정) ---
print("\n[2] 로봇팔 생성 및 관절 이동...")
try:
    # [수정됨] DH 파라미터 파일을 지정하여 로봇 객체 생성
    robot = RobotArm(num_axes=6, dh_param_file='rb5_850_dh.csv')
except FileNotFoundError as e:
    print(e)
    print("스크립트를 종료합니다. create_dh_csv.py를 먼저 실행해 주세요.")
    exit()

# 로봇 관절 각도를 'Home' 자세 (예시)로 설정 (단위: 도)
joint_angles = [0, 0, -90, 0, -90, 0]
robot.set_joint_angles(joint_angles)


# --- 3. 로봇 엔드 이펙터(EE) 포즈 계산 (실제 FK) ---
print("\n[3] 로봇 EE 포즈 계산 (Base 기준)...")
# [수정됨] 이제 이 함수는 '가짜'가 아닌 '실제' 순기구학을 계산합니다.
T_base_ee = robot.get_end_effector_pose()

print("--- 로봇 EE 포즈 (T_base_ee) ---")
print(T_base_ee)


# --- 4. (Goal 1) 검출된 객체 포즈 변환 시뮬레이션 ---
print("\n[4] 카메라가 검출한 객체 포즈 변환...")
# 가상: 카메라가 자신의 좌표계 기준으로 (z=0.8m) 앞에 있고,
# y축으로 10도 기울어진 물체를 검출했다고 가정합니다.
T_cam_object = Transform3D.from_xyz_rpy(x=0.0, y=0.0, z=0.8,
                                        rx=0, ry=10, rz=0, degrees=True)

print("--- (카메라 기준) 검출된 객체 포즈 (T_cam_object) ---")
# 이 객체의 '중심점(Translation)'과 '회전(Euler Angles)'을 보여줍니다.
print(T_cam_object)

# 이제 로봇이 이 물체를 집을 수 있도록 '로봇 베이스' 기준으로 변환합니다.
T_base_object = camera.transform_pose_from_camera_to_base_frame(T_cam_object)

print("\n--- (로봇 베이스 기준) 변환된 객체 포즈 (T_base_object) ---")
# 로봇에게 "여기로 가서 이 자세로 물건을 집어라"라고 명령할 수 있는 좌표입니다.
print(T_base_object)
print("-> 로봇은 이 (x,y,z)와 (rx,ry,rz) 값을 타겟으로 역기구학(IK)을 계산해야 합니다.")
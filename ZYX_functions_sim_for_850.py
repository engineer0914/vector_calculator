# functions.py

import numpy as np
from scipy.spatial.transform import Rotation as R
import pandas as pd
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Transform3D:
    """
    3D 공간의 변환(위치와 회전)을 나타내는 4x4 동차 변환 행렬 클래스.
    """
    
    def __init__(self, matrix):
        """
        4x4 numpy 배열로 객체를 초기화합니다.
        """
        if not isinstance(matrix, np.ndarray) or matrix.shape != (4, 4):
            raise ValueError("입력값은 4x4 numpy 배열이어야 합니다.")
        self.matrix = matrix

    @staticmethod
    def from_xyz_rpy(x, y, z, rx, ry, rz, degrees=True):
        """
        [정적 메서드]
        x, y, z 이동과 'xyz' 순서의 오일러 각(Roll, Pitch, Yaw)으로 
        새로운 Transform3D 객체를 생성합니다.
        """
        rotation = R.from_euler('xyz', [rx, ry, rz], degrees=degrees)
        rot_matrix = rotation.as_matrix()
        
        transform_matrix = np.identity(4)
        transform_matrix[0:3, 0:3] = rot_matrix
        transform_matrix[0:3, 3] = [x, y, z]
        
        return Transform3D(transform_matrix)

    @staticmethod
    def identity():
        """
        [정적 메서드]
        단위 행렬(아무 변환도 하지 않음)을 가진 Transform3D 객체를 생성합니다.
        """
        return Transform3D(np.identity(4))

    def get_translation(self):
        """
        변환 행렬에서 이동(translation) 벡터 [x, y, z]를 추출합니다.
        """
        return self.matrix[0:3, 3]

    def get_rotation_matrix(self):
        """
        변환 행렬에서 3x3 회전 행렬(Rotation Matrix)을 추출합니다.
        """
        return self.matrix[0:3, 0:3]

    def get_euler_angles(self, sequence='xyz', degrees=True):
        """
        3x3 회전 행렬에서 오일러 각 [rx, ry, rz]를 추출합니다.
        """
        rotation = R.from_matrix(self.get_rotation_matrix())
        return rotation.as_euler(sequence, degrees=degrees)

    def inverse(self):
        """
        변환의 역행렬을 계산하여 새 Transform3D 객체로 반환합니다.
        (예: T_A_B -> T_B_A)
        """
        R_inv = self.get_rotation_matrix().T
        t = self.get_translation()
        t_inv = -R_inv @ t
        
        inv_matrix = np.identity(4)
        inv_matrix[0:3, 0:3] = R_inv
        inv_matrix[0:3, 3] = t_inv
        
        return Transform3D(inv_matrix)

    def __matmul__(self, other):
        """
        행렬 곱 연산자(@)를 오버로딩합니다. (예: T_A_B @ T_B_C = T_A_C)
        두 Transform3D 객체의 변환을 연결(chain)합니다.
        """
        if not isinstance(other, Transform3D):
            raise TypeError("Transform3D 객체와만 행렬 곱(@)이 가능합니다.")
        
        new_matrix = self.matrix @ other.matrix
        return Transform3D(new_matrix)

    def __str__(self):
        """
        print() 함수로 객체를 출력할 때의 형식을 지정합니다.
        (소수점 2자리, 고정 소수점 표기)
        """
        trans = self.get_translation()
        euler = self.get_euler_angles()
        
        # Translation과 Euler Angles를 소수점 2자리(f)로 포맷팅
        trans_str = f"[{trans[0]:.2f}, {trans[1]:.2f}, {trans[2]:.2f}]"
        euler_str = f"[{euler[0]:.2f}, {euler[1]:.2f}, {euler[2]:.2f}]"
        
        # 4x4 행렬을 numpy.array2string을 사용해 포맷팅
        # formatter: 모든 float를 "0.2f" (소수점 2자리 고정) 형식으로 강제
        matrix_str_np = np.array2string(self.matrix, 
                                        precision=2, 
                                        suppress_small=True, 
                                        formatter={'float_kind': lambda x: f"{x:0.2f}"})
        
        return (f"Transform3D:\n"
                f"  Translation (x,y,z): {trans_str}\n"
                f"  Euler Angles (rx,ry,rz): {euler_str} (deg)\n"
                f"  4x4 Matrix:\n{matrix_str_np}")

    
    def get_origin(self):
        """이 변환의 원점(이동 벡터)을 반환합니다."""
        return self.matrix[0:3, 3]

    def get_axes_vectors(self, scale=0.1):
        """
        이 변환의 X, Y, Z 축 벡터를 반환합니다.
        (시각화를 위해 원점에서 뻗어나가는 벡터)
        """
        origin = self.get_origin()
        rot_mat = self.get_rotation_matrix()
        
        # x-축 (빨강)
        x_axis = origin + rot_mat[:, 0] * scale
        # y-축 (초록)
        y_axis = origin + rot_mat[:, 1] * scale
        # z-축 (파랑)
        z_axis = origin + rot_mat[:, 2] * scale
        
        return origin, x_axis, y_axis, z_axis

# -----------------------------------------------------------
# RobotArm 클래스
# -----------------------------------------------------------
class RobotArm:
    """
    DH 파라미터를 기반으로 순기구학을 계산하는 6축 로봇팔 클래스.
    """
    def __init__(self, num_axes=6, dh_param_file='rb5_850_dh.csv'):
        self.num_axes = num_axes
        self.joint_angles = np.zeros(num_axes) # (단위: 도)
        self.base_pose = Transform3D.identity() 

        if not os.path.exists(dh_param_file):
            raise FileNotFoundError(f"DH 파라미터 파일 '{dh_param_file}'을 찾을 수 없습니다. "
                                  f"create_dh_csv.py를 먼저 실행하세요.")
            
        self.dh_table = pd.read_csv(dh_param_file)
        print(f"🤖 {dh_param_file}에서 DH 파라미터를 로드하여 {self.num_axes}축 로봇팔을 생성했습니다.")

    def set_joint_angles(self, angles):
        """
        로봇의 6개 관절 각도를 설정합니다. (단위: 도)
        """
        if len(angles) != self.num_axes:
            raise ValueError(f"관절 각도는 {self.num_axes}개여야 합니다.")
        self.joint_angles = np.array(angles)
        print(f"로봇 관절 각도 설정됨: {self.joint_angles} (deg)")

    def _create_T_matrix(self, theta_deg, d, a, alpha_deg):
        """
        [비공개 메서드]
        Standard DH 파라미터 1줄로 4x4 변환 행렬(T) 1개를 생성합니다.
        """
        # 모든 각도를 라디안으로 변환
        th_rad = np.deg2rad(theta_deg)
        al_rad = np.deg2rad(alpha_deg)
        
        cos_th = np.cos(th_rad)
        sin_th = np.sin(th_rad)
        cos_al = np.cos(al_rad)
        sin_al = np.sin(al_rad)
        
        # Standard DH 공식
        T = np.array([
            [cos_th, -sin_th * cos_al,  sin_th * sin_al, a * cos_th],
            [sin_th,  cos_th * cos_al, -cos_th * sin_al, a * sin_th],
            [     0,           sin_al,           cos_al,          d],
            [     0,                0,                0,          1]
        ])
        return T

    def get_end_effector_pose(self):
        """
        현재 관절 각도를 기준으로 실제 순기구학(Forward Kinematics)을 계산합니다.
        T_0_n = T_0_1 @ T_1_2 @ ... @ T_(n-1)_n
        
        참고: 이 메서드는 get_all_link_poses()의 마지막 요소를 반환하는 것과 같습니다.
        """
        all_poses = self.get_all_link_poses()
        return all_poses[-1] # 마지막 링크(EE)의 포즈 반환
    
    def get_all_link_poses(self):
        """
        [시각화용]
        각 링크의 끝 지점(관절 조인트 또는 엔드 이펙터)의 
        로봇 베이스 좌표계 기준 Transform3D 포즈를 리스트로 반환합니다.
        """
        link_poses = []
        # T_current는 numpy 배열로 누적 계산
        T_current = self.base_pose.matrix 
        
        joint_map = {
            'theta1': self.joint_angles[0],
            'theta2': self.joint_angles[1],
            'theta3': self.joint_angles[2],
            'theta4': self.joint_angles[3],
            'theta5': self.joint_angles[4],
            'theta6': self.joint_angles[5],
            '0': 0  # 고정 링크용
        }
        
        for _, row in self.dh_table.iterrows():
            # CSV에서 파라미터 읽기
            theta_var = row['theta_var']
            theta_offset = row['theta_offset_deg']
            d = row['d']
            a = row['a']
            alpha = row['alpha_deg']
            
            # 현재 링크의 최종 theta 각도 계산
            current_theta_deg = joint_map[theta_var] + theta_offset
            
            # 이 링크의 변환 행렬 T_i-1_i 계산
            T_link = self._create_T_matrix(current_theta_deg, d, a, alpha)
            
            # 누적 곱 (월드->베이스 @ ... @ 링크i)
            T_current = T_current @ T_link
            
            # Transform3D 객체로 래핑하여 리스트에 추가
            link_poses.append(Transform3D(T_current))
            
        return link_poses # 각 링크의 끝점(조인트) 포즈 (베이스 기준)

# -----------------------------------------------------------
# Camera 클래스
# -----------------------------------------------------------
class Camera:
    """
    카메라를 나타내는 클래스.
    로봇 베이스 좌표계 기준 카메라의 상대 위치(보정 행렬)를 가집니다.
    """
    def __init__(self, T_base_to_cam: Transform3D):

        # T_base_cam: 로봇 베이스를 기준으로 본 카메라의 포즈
        self.T_base_cam = T_base_to_cam

        # T_cam_base: 카메라 기준 로봇 베이스의 포즈 (역변환)
        self.T_cam_base = T_base_to_cam.inverse()

        print("📷 카메라가 생성되고 위치에 대한 행렬 생성.")

    def transform_pose_from_base_to_camera_frame(self, T_base_object: Transform3D):
        """
        '로봇 베이스' 기준의 객체 포즈(T_base_object)를
        '카메라' 기준의 객체 포즈(T_cam_object)로 변환합니다.
        
        계산: T_cam_object = T_cam_base @ T_base_object
        """
        T_cam_object = self.T_cam_base @ T_base_object
        return T_cam_object

    def transform_pose_from_camera_to_base_frame(self, T_cam_object: Transform3D):
        """
        '카메라' 기준의 객체 포즈(T_cam_object)를
        '로봇 베이스' 기준의 객체 포즈(T_base_object)로 변환합니다.
        (로봇이 물체를 집기 위해 이 좌표가 필요합니다)
        
        계산: T_base_object = T_base_cam @ T_cam_object
        """
        T_base_object = self.T_base_cam @ T_cam_object
        return T_base_object
# functions.py

import numpy as np
from scipy.spatial.transform import Rotation as R
import pandas as pd  # pandas 임포트 추가
import os  # 파일 존재 확인을 위해 추가

class Transform3D:
    """
    3D 공간의 변환(위치와 회전)을 나타내는 4x4 동차 변환 행렬 클래스.
    (이 클래스는 이전과 동일하며, 수정할 필요가 없습니다.)
    """
    
    def __init__(self, matrix):
        if not isinstance(matrix, np.ndarray) or matrix.shape != (4, 4):
            raise ValueError("입력값은 4x4 numpy 배열이어야 합니다.")
        self.matrix = matrix

    @staticmethod
    def from_xyz_rpy(x, y, z, rx, ry, rz, degrees=True):
        rotation = R.from_euler('xyz', [rx, ry, rz], degrees=degrees)
        rot_matrix = rotation.as_matrix()
        
        transform_matrix = np.identity(4)
        transform_matrix[0:3, 0:3] = rot_matrix
        transform_matrix[0:3, 3] = [x, y, z]
        
        return Transform3D(transform_matrix)

    @staticmethod
    def identity():
        return Transform3D(np.identity(4))

    def get_translation(self):
        return self.matrix[0:3, 3]

    def get_rotation_matrix(self):
        return self.matrix[0:3, 0:3]

    def get_euler_angles(self, sequence='xyz', degrees=True):
        rotation = R.from_matrix(self.get_rotation_matrix())
        return rotation.as_euler(sequence, degrees=degrees)

    def inverse(self):
        R_inv = self.get_rotation_matrix().T
        t = self.get_translation()
        t_inv = -R_inv @ t
        
        inv_matrix = np.identity(4)
        inv_matrix[0:3, 0:3] = R_inv
        inv_matrix[0:3, 3] = t_inv
        
        return Transform3D(inv_matrix)

    def __matmul__(self, other):
        if not isinstance(other, Transform3D):
            raise TypeError("Transform3D 객체와만 행렬 곱(@)이 가능합니다.")
        
        new_matrix = self.matrix @ other.matrix
        return Transform3D(new_matrix)

    def __str__(self):
        trans = self.get_translation()
        euler = self.get_euler_angles()
        
        return (f"Transform3D:\n"
                f"  Translation (x,y,z): [{trans[0]:.3f}, {trans[1]:.3f}, {trans[2]:.3f}]\n"
                f"  Euler Angles (rx,ry,rz): [{euler[0]:.3f}, {euler[1]:.3f}, {euler[2]:.3f}] (deg)\n"
                f"  4x4 Matrix:\n{np.round(self.matrix, 3)}")

# -----------------------------------------------------------
# [수정됨] RobotArm 클래스
# -----------------------------------------------------------
class RobotArm:
    """
    DH 파라미터를 기반으로 순기구학을 계산하는 6축 로봇팔 클래스.
    """
    def __init__(self, num_axes=6, dh_param_file='rb5_850_dh.csv'):
        self.num_axes = num_axes
        self.joint_angles = np.zeros(num_axes) # (단위: 도)
        self.base_pose = Transform3D.identity() 
        
        # DH 파라미터 로드
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
        (이미지 2의 행렬 공식 기반)
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
        [수정됨]
        현재 관절 각도를 기준으로 실제 순기구학(Forward Kinematics)을 계산합니다.
        T_0_n = T_0_1 @ T_1_2 @ ... @ T_(n-1)_n
        """
        
        # 1. 6개 관절 각도를 DH 테이블의 'theta_var'에 매핑
        joint_map = {
            'theta1': self.joint_angles[0],
            'theta2': self.joint_angles[1],
            'theta3': self.joint_angles[2],
            'theta4': self.joint_angles[3],
            'theta5': self.joint_angles[4],
            'theta6': self.joint_angles[5],
            '0': 0  # 고정 링크용
        }
        
        # 2. 모든 변환 행렬을 순차적으로 곱합니다.
        T_total = np.identity(4)
        
        for _, row in self.dh_table.iterrows():
            # CSV에서 파라미터 읽기
            theta_var = row['theta_var']
            theta_offset = row['theta_offset_deg']
            d = row['d']
            a = row['a']
            alpha = row['alpha_deg']
            
            # 현재 링크의 최종 theta 각도 계산
            # (변수 각도 + 오프셋 각도)
            current_theta_deg = joint_map[theta_var] + theta_offset
            
            # 이 링크의 변환 행렬 T_i-1_i 계산
            T_link = self._create_T_matrix(current_theta_deg, d, a, alpha)
            
            # 누적 곱
            T_total = T_total @ T_link
        
        # 3. 최종 행렬을 Transform3D 객체로 래핑하여 반환
        # (베이스 좌표계 기준 EE 포즈)
        pose_in_base_frame = Transform3D(T_total)
        
        # 월드 좌표계 기준 EE 포즈 (월드->베이스 @ 베이스->EE)
        pose_in_world_frame = self.base_pose @ pose_in_base_frame
        return pose_in_world_frame

# -----------------------------------------------------------
# [수정됨] Camera 클래스
# -----------------------------------------------------------
class Camera:
    """
    카메라를 나타내는 클래스.
    로봇 베이스 좌표계 기준 카메라의 상대 위치(보정 행렬)를 가집니다.
    """
    def __init__(self, T_base_to_cam: Transform3D):
        # T_base_cam: 로봇 베이스 기준 카메라의 포즈 (Extrinsic)
        self.T_base_cam = T_base_to_cam
        
        # T_cam_base: 카메라 기준 로봇 베이스의 포즈 (역변환)
        self.T_cam_base = T_base_to_cam.inverse()
        print("📷 카메라가 생성되고 보정 행렬이 설정되었습니다.")

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
        [추가됨 - Goal 1]
        '카메라' 기준의 객체 포즈(T_cam_object)를
        '로봇 베이스' 기준의 객체 포즈(T_base_object)로 변환합니다.
        (로봇이 물체를 집기 위해 이 좌표가 필요합니다)
        
        계산: T_base_object = T_base_cam @ T_cam_object
        """
        T_base_object = self.T_base_cam @ T_cam_object
        return T_base_object
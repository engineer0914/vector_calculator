import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 필수 라이브러리 임포트 (없으면 에러 발생)
try:
    import rbpodo as rb
    from functions_sim_for_850 import Transform3D, RobotArm, Camera
except ImportError as e:
    print(f"❌ 필수 라이브러리가 없습니다: {e}")
    print("rbpodo가 설치되어 있고, functions_sim_for_850.py 파일이 같은 폴더에 있는지 확인해주세요.")
    sys.exit(1)

# =========================================================
# 환경 설정
# =========================================================
ROBOT_IP = "10.0.2.7"      # 실제 로봇 IP
DH_FILE = 'rb3_730_dh.csv' # DH 파라미터 파일

# =========================================================
# 헬퍼 함수
# =========================================================
def to_6d_array(transform: Transform3D):
    """Transform3D 객체를 로봇 입력용 [x,y,z,rx,ry,rz] 배열로 변환"""
    xyz = transform.get_translation()
    rpy = transform.get_euler_angles(sequence='xyz', degrees=True)
    return np.array([xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]])

def draw_frame(ax, T, label, scale=50, lw=2):
    """3D 그래프에 좌표계 그리기"""
    o, x, y, z = T.get_axes_vectors(scale)
    ax.scatter(*o, color='k', s=20)
    ax.quiver(*o, *(x-o), color='r', lw=lw) # X축
    ax.quiver(*o, *(y-o), color='g', lw=lw) # Y축
    ax.quiver(*o, *(z-o), color='b', lw=lw) # Z축
    if label:
        ax.text(o[0], o[1], o[2], label, fontsize=9)

# =========================================================
# 메인 함수
# =========================================================
def main():
    print("--- [통합 스크립트] 시뮬레이션 후 실제 로봇 구동 ---")

    # -----------------------------------------------------
    # [Step 1] 시뮬레이션: 좌표 계산 및 3D 시각화
    # -----------------------------------------------------
    print("\n📊 [Step 1] 시뮬레이션 및 좌표 계산 시작...")

    # 1-1. 로봇 모델 생성 및 초기 자세 설정
    if not os.path.exists(DH_FILE):
        print(f"❌ {DH_FILE} 파일을 찾을 수 없습니다.")
        return
    robot_sim = RobotArm(num_axes=6, dh_param_file=DH_FILE)
    
    # 요청하신 초기 관절 각도 설정
    init_joint_angles = [0, 0, -90, -90, 0, 0]
    robot_sim.set_joint_angles(init_joint_angles)
    
    # 초기 자세에서의 EE 포즈 계산 (이것이 실제 구동의 '현재 자세'가 됨)
    T_base_ee_start = robot_sim.get_end_effector_pose()
    print(f"   -> 초기 EE 포즈 계산 완료: {T_base_ee_start.get_translation().round(2)}")

    # 1-2. 오프셋 설정 (카메라 & 그리퍼)
    T_ee_cam = Transform3D.from_xyz_rpy(x=50.0, y=0.0, z=50.0, rx=0, ry=0, rz=0)
    T_base_cam = T_base_ee_start @ T_ee_cam
    camera_sim = Camera(T_base_cam)

    T_ee_gripper = Transform3D.from_xyz_rpy(x=0.0, y=0.0, z=100.0, rx=0, ry=0, rz=0)
    T_base_gripper = T_base_ee_start @ T_ee_gripper

    # 1-3. 객체 및 진입점 계산
    # (가정) 카메라 기준 객체 위치
    T_cam_object = Transform3D.from_xyz_rpy(x=50.0, y=0.0, z=300.0, rx=0, ry=90, rz=90, degrees=True)
    # 베이스 기준 객체 위치 (목표점)
    T_base_object = camera_sim.transform_pose_from_camera_to_base_frame(T_cam_object)
    
    # 진입 오프셋 (객체 X축 기준 +200mm)
    T_obj_approach = Transform3D.from_xyz_rpy(x=200.0, y=0, z=0, rx=0, ry=0, rz=0)
    # 베이스 기준 진입점 위치 (경유점)
    T_base_approach = T_base_object @ T_obj_approach

    # 1-4. 3D 시각화 (그래프 창을 닫으면 다음으로 진행됨)
    print("   -> 🖥️ 3D 시뮬레이션 그래프를 표시합니다. (확인 후 창을 닫아주세요)")
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title('Simulation View (Close this window to move real robot)')
    ax.set_xlabel('X [mm]'); ax.set_ylabel('Y [mm]'); ax.set_zlabel('Z [mm]')
    limit = 800; ax.set_xlim([-limit, limit]); ax.set_ylim([-limit, limit]); ax.set_zlim([0, limit*1.5])
    ax.view_init(elev=30, azim=45)

    # 로봇, 카메라, 그리퍼 그리기
    link_poses = robot_sim.get_all_link_poses()
    points = [np.zeros(3)] + [p.get_origin() for p in link_poses]
    xs, ys, zs = zip(*points)
    ax.plot(xs, ys, zs, 'o-', color='gray', lw=3, label='Robot Arm')
    draw_frame(ax, T_base_ee_start, 'EE(Start)', scale=60)
    draw_frame(ax, T_base_cam, 'Camera', scale=60)
    draw_frame(ax, T_base_gripper, 'Gripper', scale=60)

    # 객체, 진입점, 경로 그리기
    draw_frame(ax, T_base_object, 'Object', scale=80)
    ox, oy, oz = T_base_object.get_origin()
    ax.scatter(ox, oy, oz, color='orange', s=150, marker='s', label='Object')

    draw_frame(ax, T_base_approach, 'Approach', scale=60)
    ap_x, ap_y, ap_z = T_base_approach.get_origin()
    ax.scatter(ap_x, ap_y, ap_z, color='red', s=100, marker='x', label='Approach Point')

    # 진입 경로 (Approach -> Object)
    ax.plot([ap_x, ox], [ap_y, oy], [ap_z, oz], '--', color='red', lw=2, label='Approach Path')

    ax.legend()
    plt.show() # 여기서 코드 실행이 일시 중지됨 (창 닫을 때까지)
    print("   -> 시뮬레이션 종료. 실제 로봇 구동 준비.")


    # -----------------------------------------------------
    # [Step 2] 실제 로봇 구동: 포인트 블렌딩 이동
    # -----------------------------------------------------
    print("\n🤖 [Step 2] 실제 로봇 연결 및 구동 시작...")
    
    # 안전을 위한 사용자 확인 (주석 처리 가능)
    # input("⚠️ 실제 로봇이 움직입니다. 주변이 안전한지 확인 후 Enter를 누르세요...")

    try:
        # 2-1. 로봇 연결
        cobot = rb.Cobot(ROBOT_IP)
        rc = rb.ResponseCollector()
        print(f"   -> 로봇({ROBOT_IP}) 연결 성공")

        # 작동 모드 설정 (테스트 시 Simulation 권장, 실제 구동 시 Real로 변경)
        # cobot.set_operation_mode(rc, rb.OperationMode.Simulation)
        cobot.set_operation_mode(rc, rb.OperationMode.Real) 
        cobot.set_speed_bar(rc, 0.5) # 속도 50% 설정

        # 2-2. 이동 경로 포인트 준비 (6D 배열로 변환)
        # *중요*: Point Blending의 시작점은 '현재 로봇 자세'입니다.
        # 시뮬레이션에서 설정한 초기 자세를 현재 자세로 가정하고 계산한 값을 사용합니다.
        # p_start는 참고용이며 move_pb_add에 추가하지 않습니다.
        p_start_sim = to_6d_array(T_base_ee_start)
        
        # 경유점 1: 진입점 (Approach Point)
        p_approach = to_6d_array(T_base_approach)
        
        # 도착점: 객체 위치 (Object Point)
        p_object = to_6d_array(T_base_object)

        print(f"\n   [이동 경로 정보 (TCP 기준)]")
        print(f"   1. 현재 자세 (가정): {p_start_sim.round(2)}")
        print(f"   👉 2. 진입점 (경유): {p_approach.round(2)}")
        print(f"   👉 3. 객체 위치 (도착): {p_object.round(2)}")

        # 2-3. Point Blending 명령 생성
        cobot.move_pb_clear(rc)

        # Point 1: 진입점 (Approach)으로 이동
        # 속도: 400mm/s, 블렌딩: 50% (부드럽게 통과)
        cobot.move_pb_add(rc, p_approach, 400.0, rb.BlendingOption.Ratio, 0.5)

        # Point 2: 객체 위치 (Object)로 진입
        # 속도: 100mm/s (정밀 진입), 블렌딩: 0% (정확히 멈춤)
        cobot.move_pb_add(rc, p_object, 100.0, rb.BlendingOption.Ratio, 0.0)

        # 2-4. 명령 전송 및 실행
        print("\n   -> 명령 버퍼 전송 (Flush)...")
        cobot.flush(rc)
        rc = rc.error().throw_if_not_empty()
        rc.clear()

        print("   -> 🚀 로봇 이동 시작 (MovePB)...")
        # 가속 시간: 800ms, 옵션: 의도된 경로 유지
        cobot.move_pb_run(rc, 800.0, rb.MovePBOption.Intended)

        # 2-5. 완료 대기
        if cobot.wait_for_move_started(rc, 0.1).type() == rb.ReturnType.Success:
            print("   -> 이동 중...")
            cobot.wait_for_move_finished(rc)
            print("   -> ✨ 이동 완료!")
        else:
            print("   -> ❌ 이동 시작 실패 (타임아웃 또는 에러)")
            
        # 최종 에러 체크
        rc = rc.error().throw_if_not_empty()

    except Exception as e:
        print(f"\n❌ [Error] 로봇 구동 중 오류 발생:\n{e}")
    finally:
        # 필요하다면 연결 종료 처리 (SDK에 따라 다름)
        print("\n--- 스크립트 종료 ---")

if __name__ == "__main__":
    main()
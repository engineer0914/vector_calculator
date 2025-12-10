# 개발중인 코드
# ASYNC 를 이용한 다중 태스크 실행



import logging
import asyncio
import rbpodo as rb
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 작성하신 functions.py에서 클래스 가져오기
from functions_sim_for_850 import Transform3D, RobotArm

logging.basicConfig(format='%(asctime)s.%(msecs)03d %(levelname)s %(message)s',
                    datefmt='%Y-%m-%d,%H:%M:%S',
                    level=logging.INFO)

ROBOT_IP = "192.168.0.100"

class GLOBAL:
    running = True
    q = np.zeros((6,)) # 현재 관절 각도 (실시간 업데이트)

# --- 시각화 초기 설정 ---
plt.ion() # [핵심] Interactive Mode 켜기 (창이 안 닫히고 계속 갱신됨)
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# 로봇 객체 미리 생성 (CSV 파일 필요)
try:
    # DH 파라미터 파일 이름이 맞는지 확인하세요
    sim_robot = RobotArm(num_axes=6, dh_param_file='rb5_850_dh.csv')
except Exception as e:
    print(f"Error: {e}")
    exit()

def update_plot_frame(joint_angles):
    """
    현재 각도를 받아서 그래프를 싹 지우고 다시 그리는 함수
    """
    ax.cla()

    # 1. 로봇 자세 업데이트
    sim_robot.set_joint_angles(joint_angles)
    poses = sim_robot.get_all_link_poses()

    # 2. 좌표 추출 및 그리기
    xs, ys, zs = [], [], []
    
    # Base
    origin = sim_robot.base_pose.get_translation()
    xs.append(origin[0])
    ys.append(origin[1])
    zs.append(origin[2])

    for pose in poses:
        trans = pose.get_translation()
        xs.append(trans[0])
        ys.append(trans[1])
        zs.append(trans[2])
        
        # 좌표축 그리기 (속도를 위해 생략 가능, 여기선 포함)
        o, x_axis, y_axis, z_axis = pose.get_axes_vectors(scale=0.1) # scale 조절 필요 (m 단위면 0.1, mm면 100)
        ax.plot([o[0], x_axis[0]], [o[1], x_axis[1]], [o[2], x_axis[2]], 'r-', lw=1)
        ax.plot([o[0], y_axis[0]], [o[1], y_axis[1]], [o[2], y_axis[2]], 'g-', lw=1)
        ax.plot([o[0], z_axis[0]], [o[1], z_axis[1]], [o[2], z_axis[2]], 'b-', lw=1)

    # 링크 연결선
    ax.plot(xs, ys, zs, 'o-', color='black', linewidth=3, markersize=6)

    # 3. 그래프 범위 및 라벨 고정 (지우고 다시 그리므로 매번 설정해야 함)
    ax.set_xlim(-0.8, 0.8) # 로봇 크기에 맞춰 조절 (m 단위 가정)
    ax.set_ylim(-0.8, 0.8)
    ax.set_zlim(0, 1.0)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'Real-time Robot Pose\nJoints: {np.round(joint_angles, 1)}')

    # 4. 화면 갱신
    plt.draw()
    plt.pause(0.001)

# --- Async Task 정의 ---

async def get_data():
    """로봇 데이터를 받아오는 태스크"""
    data_channel = rb.asyncio.CobotData(ROBOT_IP)
    print("Connecting to Data Channel...")

    while GLOBAL.running:
        try:
            data = await data_channel.request_data()
            if data is not None:
                GLOBAL.q = data.sdata.jnt_ref # 현재 관절 각도 업데이트
            await asyncio.sleep(0.05) # 20Hz
        except Exception as e:
            logging.error(f"Data connection error: {e}")
            break

async def visualization_loop():
    """화면을 그리는 태스크"""
    print("Starting Visualization Loop...")
    while GLOBAL.running:
        # 현재 GLOBAL에 저장된 최신 각도로 그림 그리기
        update_plot_frame(GLOBAL.q)
        
        # 너무 자주 그리면 느려지므로 0.1초(10FPS) 정도가 적당
        await asyncio.sleep(0.1) 

async def move_thread():
    """로봇을 움직이는 태스크 (기존 코드 유지)"""
    robot = rb.asyncio.Cobot(ROBOT_IP)
    rc = rb.ResponseCollector()
    
    # ... (기존 콜백 설정 등 생략) ...

    await robot.set_operation_mode(rc, rb.OperationMode.Simulation)
    await robot.set_speed_bar(rc, 0.5)

    logging.info("Move Thread Started.")
    
    try:
        # 테스트: 0도로 이동
        await robot.move_j(rc, np.array([0, 0, 0, 0, 0, 0]), 100, 200)
        await asyncio.sleep(2)
        
        # 테스트: 움직임 추가 (예: 90도로 굽히기)
        logging.info("Moving to pose 2...")
        await robot.move_j(rc, np.array([0, 0, 90, 0, 90, 0]), 100, 200)
        await asyncio.sleep(3)

        # 다시 홈으로
        logging.info("Moving Home...")
        await robot.move_j(rc, np.array([0, 0, 0, 0, 0, 0]), 100, 200)
        await asyncio.sleep(2)

    except Exception as e:
        logging.error(f"Robot Control Error: {e}")
    
    finally:
        logging.info("Robot task finished.")
        GLOBAL.running = False # 전체 종료 신호

async def _main():
    # 세 개의 태스크를 동시에 실행
    # 1. 데이터 수신 (Data)
    # 2. 로봇 제어 (Control)
    # 3. 화면 그리기 (Vis)
    
    task_data = asyncio.create_task(get_data())
    task_vis = asyncio.create_task(visualization_loop())
    task_move = asyncio.create_task(move_thread())

    await asyncio.gather(task_data, task_vis, task_move)

if __name__ == "__main__":
    try:
        asyncio.run(_main())
    except KeyboardInterrupt:
        print("Finished.")
    finally:
        plt.close('all')

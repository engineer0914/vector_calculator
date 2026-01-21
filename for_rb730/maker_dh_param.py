import pandas as pd
import io

print("RB3-730 DH 파라미터(Standard Spong) CSV 파일을 생성합니다...")

# [데이터 출처 및 분석]
# 1. joint.yaml 분석
#    - Base Z (d1): 145.3 mm
#    - Elbow Y (d2): -6.45 mm (어깨/팔꿈치 오프셋) -> 구조적 특징 반영
#    - Elbow Z (a1): 286.0 mm (상완 길이) -> 0이 아님! 확인 완료.
#    - Wrist2 Z (d4): 344.0 mm (하완 길이) -> Wrist1 축방향이므로 d 파라미터로 들어감
# 2. rb3_730es_u.urdf.xacro 분석
#    - TCP Offset (d6): 100.0 mm (tcp_xyz="0 0 0.1" 등에서 추출)

# [Spong 모델링 매핑 - 2-2-2 구조]
# Row 1: Base 회전 (d=145.3)
# Row 2: Shoulder 회전 (오프셋 d=-6.45)
# Row 3: 상완 길이 (a=286.0) -> *중요: 여기가 0이 아님*
# Row 4: Elbow 회전
# Row 5: (거리 0, 90도 꺾임)
# Row 6: Wrist1 회전 (하완 길이 d=344.0 포함)
# Row 7: (거리 0, 90도 꺾임)
# Row 8: Wrist2 회전
# Row 9: Wrist3 회전 (TCP 길이 d=100.0 포함)

data = """link_id,theta_var,theta_offset_deg,d,a,alpha_deg
1,theta1,0,145.3,0,-90
2,theta2,-90,-6.45,0,0
3,0,0,0,286.0,0
4,theta3,0,0,0,90
5,0,0,0,0,0
6,theta4,0,344.0,0,-90
7,0,0,0,0,90
8,theta5,0,0,0,-90
9,theta6,0,100.0,0,0
"""

# 데이터 프레임 변환
df = pd.read_csv(io.StringIO(data))

# CSV 저장
file_name = 'rb3_730_dh.csv'
df.to_csv(file_name, index=False)

print(f"'{file_name}' 파일 생성 완료.")
print("-" * 30)
print("생성된 DH 파라미터 (RB3-730):")
print(df)
print("-" * 30)
print("검토 결과: 상완(a=286.0), 하완(d=344.0), 오프셋(d=-6.45) 모두 포함됨.")
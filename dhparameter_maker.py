import pandas as pd
import io

print("RB5-850 DH 파라미터(Standard Spong) CSV 파일을 생성합니다...")

# 이미지 1의 "Standard(Spong)" 테이블 구조
# 이미지 2의 "RB5-850" 테이블 값 (d1~d6, a1, a2)
# d1=169.2, d2=148.4, d3=148.4, d4=110.7, d5=110.7, d6=96.7
# a1=425.0, a2=392.0
# (참고: 이미지 2의 테이블에서 RB5-850 행을 사용)

# DH 파라미터 데이터를 정의합니다.
# Spong 모델은 6축 로봇을 9 링크로 표현합니다.
# theta_var: 'theta1'~'theta6'는 변수, '0'은 고정 상수를 의미
# theta_offset_deg: 변수에 더해지는 각도 오프셋 (단위: 도)
data = """link_id,theta_var,theta_offset_deg,d,a,alpha_deg
1,theta1,0,169.2,0,-90
2,theta2,-90,-148.4,0,0
3,0,0,0,425.0,0
4,theta3,0,148.4,0,0
5,0,0,0,392.0,0
6,theta4,90,-110.7,0,0
7,0,0,0,0,-90
8,theta5,0,110.7,0,-90
9,theta6,0,-96.7,0,90
"""

# 데이터를 DataFrame으로 읽어들입니다.
df = pd.read_csv(io.StringIO(data))

# CSV 파일로 저장합니다.
file_name = 'rb5_850_dh.csv'
df.to_csv(file_name, index=False)

print(f"'{file_name}' 파일이 성공적으로 생성되었습니다.")
print("생성된 DH 파라미터:")
print(df)
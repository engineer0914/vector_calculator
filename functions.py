# 1. 클래스 (Class) 정의: '자동차'라는 설계도를 만듭니다.
class Car:

    def __init__(self, model_name, color):
        self.model = model_name
        self.color = color
        self.speed = 0
        print(f"🎉 {self.color}색 {self.model} 차량이 생성되었습니다.")

    def accelerate(self, amount):
        self.speed += amount

    def brake(self, amount):
        self.speed -= amount
        if self.speed < 0:
            self.speed = 0
        print(f"끼익! 현재 속도: {self.speed} km/h")

    def show_status(self):
        print(f"현재 차량 정보: [모델: {self.model}, 색상: {self.color}, 속도: {self.speed} km/h]")




class Robotarm:

    def __init__(self, model_name, color):
        self.model = model_name  # 'self'는 객체 자기 자신을 가리킵니다.
        self.color = color
        self.speed = 0           # 모든 차는 처음엔 속도가 0입니다.
        print(f"🎉 {self.color}색 {self.model} 차량이 생성되었습니다.")






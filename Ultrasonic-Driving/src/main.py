#!/usr/bin/env python
# -*- coding: utf-8 -*- 2



#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import rospy
import numpy as np
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
motor = None  # 모터 노드 변수
Fix_Speed = 5  # 모터 속도 고정 상수값 
new_angle = 0  # 모터 조향각 초기값
new_speed = Fix_Speed  # 모터 속도 초기값
ultra_msg = None  # 초음파 데이터를 담을 변수
ultra_data = None  # 초음파 토픽의 필터링에 사용할 변수
motor_msg = xycar_motor()  # 모터 토픽 메시지

#=============================================
# 프로그램에서 사용할 이동평균필터 클래스
#=============================================
class MovingAverage:

    def __init__(self, n):
        self.samples = n
        self.data = []
        self.weights = list(range(1, n + 1))

    def add_sample(self, new_sample):
        if len(self.data) < self.samples:
            self.data.append(new_sample)
        else:
            self.data = self.data[1:] + [new_sample]
            
    def get_sample_count(self):
        return len(self.data)
        
    # 이동평균값을 구하는 함수
    def get_mavg(self):
        return float(sum(self.data)) / len(self.data)

    # 중앙값을 사용해서 이동평균값을 구하는 함수
    def get_mmed(self):
        return float(np.median(self.data))

    # 가중치를 적용하여 이동평균값을 구하는 함수        
    def get_wmavg(self):
        s = 0
        for i, x in enumerate(self.data):
            s += x * self.weights[i]
        return float(s) / sum(self.weights[:len(self.data)])
        
#=============================================
# 초음파 8개의 거리정보에 대해서 이동평균필터를 적용하기 위한 선언
#=============================================
avg_count = 5  # 이동평균값을 계산할 데이터 묶음 갯수
ultra_mvavg = [MovingAverage(avg_count) for i in range(8)]

#=============================================
# 콜백함수 - 초음파 토픽을 처리하는 콜백함수.
#=============================================
def ultra_callback(data):
    global ultra_msg, ultra_data
    #ultra_msg = data.data
    ultra_data = data.data

    # 이동평균필터를 적용해서 튀는 값을 제거해서 ultra_msg_ft에 담기
    for i in range(8):
        ultra_mvavg[i].add_sample(float(ultra_data[i]))
        
    ultra_list = [int(ultra_mvavg[i].get_mmed()) for i in range(8)]
    ultra_msg = tuple(ultra_list)

#=============================================
# 모터 토픽을 발행하는 함수.  
#=============================================
def drive(angle, speed):
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)
    
#=============================================
# 초음파 센서를 이용해서 벽까지의 거리를 알아내서
# 벽과 충돌하지 않으며 주행하도록 핸들 조정함.
#=============================================



def print_ultra_msg(u_msg):
    print('left     :', u_msg[0])
    print('front[1] :', u_msg[1])
    print('front[2] :', u_msg[2])
    print('front[3] :', u_msg[3])
    print('right    :', u_msg[4])
    # print('back[5]  :', u_msg[5])
    # print('back[6]  :', u_msg[6])
    # print('back[7]  :', u_msg[7])

l_cnt = 0
r_cnt = 0

def sonic_drive():
    global new_angle, new_speed, l_cnt, r_cnt


    R = ultra_msg[3] 
    L = ultra_msg[1]  
    Q = R - L
    # print(R,L)
  
    # ================
    # How to drive ?
    # ================
    angle = Q*2  
    
    # motor_pub.publish(motor_msg)
    new_angle = int(angle)
    new_speed = 13
    if(ultra_msg[0]>120):
        l_cnt += 1
        if(l_cnt>12):
            new_angle = -50
    else:
        l_cnt=0
    if(ultra_msg[4]>120):
        r_cnt += 1
        print('shfjfukfjkfgjcfhjfkk')
        if(r_cnt>12):
            
            new_angle = 50
    else:
        r_cnt=0

    # # 앞쪽 가까이에 장애물이 있으면 차량 멈춤
    # if (min(ultra_msg[1], ultra_msg[2], ultra_msg[3]) < 5):
    #     new_angle = new_angle
    #     new_speed = 0
    #     print("Car Brake, Stop! : ")

    # # 왼쪽이 오른쪽보다 조금 멀리 있으면 있으면 작은 각도로 좌회전 주행
    # if (ultra_msg[0]-ultra_msg[4] > 10) or (ultra_msg[1]-ultra_msg[3] > 10):
    #     new_angle = -15
    #     # new_speed = Fix_Speed
    #     print("Turn left1 : ")
        
    # # 왼쪽이 오른쪽보다 많이 멀리 있으면 있으면 큰 각도로 좌회전 주행
    # elif (ultra_msg[0]-ultra_msg[4] > 20) or (ultra_msg[1]-ultra_msg[3] > 20):
    #     new_angle = -20
    #     # new_speed = Fix_Speed
    #     print("Turn left1 : ")
    
    # # 오른쪽이 왼쪽보다 조금 멀리 있으면 있으면 작은 각도로 우회전 주행
    # elif (ultra_msg[4]-ultra_msg[0] > 10) or (ultra_msg[3]-ultra_msg[1] > 10):
    #     new_angle = 15
    #     # new_speed = Fix_Speed
    #     print("Turn right1 : ")
   
    # # 오른쪽이 왼쪽보다 많이 멀리 있으면 있으면 큰 각도로 우회전 주행
    # elif (ultra_msg[4]-ultra_msg[0] > 20) or (ultra_msg[3]-ultra_msg[1] > 20):
    #     new_angle = 20
    #     # new_speed = Fix_Speed
    #     print("Turn right1 : ")
   
#    # 위 조건에 해당하지 않는 경우라면 (오른쪽과 왼쪽이 비슷한 경우) 똑바로 직진 주행
#     else:
#         new_angle = 0
#         new_speed = Fix_Speed
#         print("Go Straight : ")

    print_ultra_msg(ultra_msg)
    print('\nangle : ', new_angle)
    # 모터에 주행명령 토픽을 보낸다
    drive(new_angle, new_speed)

#=============================================
# 실질적인 메인함수 
#=============================================
def start():
    global motor
    
    #=========================================
    # 노드를 생성하고, 구독/발행할 토픽들을 선언합니다.
    #=========================================
    rospy.init_node('ultra_driver')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, ultra_callback, queue_size=1)

    #=========================================
    # 첫번째 토픽이 도착할 때까지 기다립니다.
    #=========================================
    rospy.wait_for_message("xycar_ultrasonic", Int32MultiArray)
    print("UltraSonic Ready ----------")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        # 초음파 센서로 주행합니다.
        sonic_drive()
        rate.sleep()

#=============================================
# 메인함수 - 여기서 start() 함수 호출
#=============================================
if __name__ == '__main__':
    start()















































































































#!/usr/bin/env python
#-- coding:utf-8 --

import os
import pygame, pygame.event
import numpy as np
import math

from math import atan2, degrees
from math import radians, copysign

#색상 정리
BLACK= ( 0,  0,  0)
WHITE= (255,255,255)
BLUE = (164,241,255)
GREEN= ( 0,255,  0)
RED  = (255,  0,  0)
NAVY = (70,130,180)
ORANGE = (240,128,128)
LIGHT_PINK = (255,163,212)
PURPLE = (72,62,139)
SKY_BLUE = (0, 255, 255)
PINK = (255, 0, 255)

ar_center = [1142.5,62.5]
ar_end = [1175.0,95.0]
#Path Planning 변수
MAX_T = 100.0
MIN_T = 5.0

class C:
    # PID config
    Kp = 1.0
    # System config
    k = 1.0
    MAX_STEER = np.radians(20.0)

class Map(pygame.sprite.Sprite):
    #생성자함수
    def __init__(self, screen, w, h):
        super(Map, self).__init__()
        self.screen = screen
        self.logo = pygame.image.load("logo.png").convert_alpha()
        self.font = pygame.font.SysFont('notosansmonocjkkrblack',39)
        self.font2 = pygame.font.SysFont('notosansmonocjkkrblack',45)
        self.angle = self.font.render('-45      0       45', True, WHITE)
        self.plan = self.font2.render('Planning', True, WHITE)
        self.drive = self.font2.render('Driving', True, WHITE)
    
    #Map 업데이트 함수
    def update(self, finish):
        pygame.draw.rect(self.screen, WHITE, [0,0,1190,850])
        pygame.draw.line(self.screen, BLACK, [1000,-100] , [1190,90] , 28)
        pygame.draw.line(self.screen, GREEN, [1110,30] , [1175,95] , 7)
        # 자동차 초기 위치 그리기 
        pygame.draw.rect(self.screen, BLUE, [36,118,128,64])
        pygame.draw.rect(self.screen, BLUE, [36,318,128,64])
        pygame.draw.rect(self.screen, BLUE, [36,518,128,64])
        pygame.draw.rect(self.screen, BLUE, [36,718,128,64])
        pygame.draw.rect(self.screen, BLUE, [268,636,64,128])
        pygame.draw.rect(self.screen, BLUE, [468,636,64,128])
        pygame.draw.rect(self.screen, BLUE, [668,636,64,128])
        pygame.draw.rect(self.screen, BLUE, [868,636,64,128])
        pygame.draw.rect(self.screen, BLUE, [1068,536,64,128])
        pygame.draw.rect(self.screen, BLUE, [1068,236,64,128])
        # 차량각도 버튼 
        pygame.draw.rect(self.screen, PURPLE, [750,783,64,54])
        pygame.draw.rect(self.screen, PURPLE, [820,783,64,54])
        pygame.draw.rect(self.screen, PURPLE, [890,783,64,54])
        # planning 시작 버튼
        pygame.draw.rect(self.screen, NAVY, [250,783,200,54])
        # driving 시작 버튼
        pygame.draw.rect(self.screen, ORANGE, [480,783,200,54])
        self.screen.blit(self.plan, (288,795))
        self.screen.blit(self.drive, (528,795))
        self.screen.blit(self.angle, (762,798))
        self.screen.blit(self.logo, (40,30))
        #주차 사각형 그리기
        if finish == 0 or finish == 2:
                pygame.draw.line(self.screen, RED, [1096,36], [1013, 119],4)
                pygame.draw.line(self.screen, RED, [1013, 119], [1079, 185], 4)
                pygame.draw.line(self.screen, RED, [1079, 185], [1162, 102], 4)
                pygame.draw.line(self.screen, RED, [1162, 102], [1096,36], 4)
        elif finish == 1:
                pygame.draw.line(self.screen, GREEN, [1096,36], [1013, 119],4)
                pygame.draw.line(self.screen, GREEN, [1013, 119], [1079, 185], 4)
                pygame.draw.line(self.screen, GREEN, [1079, 185], [1162, 102], 4)
                pygame.draw.line(self.screen, GREEN, [1162, 102], [1096,36], 4)
		

#자동차 클래스
class Car(pygame.sprite.Sprite):
    #생성자함수
    def __init__(self, x, y, screen, angle=0.0, max_acceleration=1000.0):
        super(Car, self).__init__()
        self.screen = screen
	# 차량의 현재 위치
        self.x = x
        self.y = y
	#yaw 값 (차량의 진행방향 == 각도)
        self.yaw = angle
	#최대 가속도 값
        self.max_acceleration = max_acceleration
	#선속도 
        self.linear_velocity = 0.0
	#최대 속도
        self.max_velocity = 100
	#조향각
        self.steering_angle = 0.0
        #자동차 휠베이스 (축거 : 앞바퀴축과 뒷바퀴축 사이의 거리)
        self.wheel_base = 84

	#자동차 이미지 좌표 (가로x세로 128x64 픽셀의 자동차 그림파일. car.png)
        self.car_img_x = 0
        self.car_img_y = 0
        
        self.car_x_ori = [-64,-64, 64, 64] # 왼쪽 위아래, 오른쪽 위아래 포인트 총4개
        self.car_y_ori = [-32, 32,-32, 32] # 왼쪽 위아래, 오른쪽 위아래 포인트 총4개
        
        self.car_x = [0,0,0,0]
        self.car_y = [0,0,0,0]

        self.car_center = [0.0,0.0]
        self.car_front_center =[0.0, 0.0]
        self.car_back_center = [0.0, 0.0]

        self.car_x_list = []
        self.car_y_list = []
	
        #차량 이미지를 불러온다.
        # convert_alpha()를 통해 RGB 채널을 RGBA 채널로 전환한다. 
        self.image = pygame.image.load("car.png").convert_alpha()
        #차량의 변위각만큼 이미지를 회전시킨다. 
        self.rotated = pygame.transform.rotate(self.image, self.yaw)
        #차량위치를 옮기면 주차완료상태를 리셋 
        game.finish = 0

    #차량 업데이트 함수
    def update(self, acceleration, delta, dt):
        delta = self.limit_input(delta)	
        #print(delta)
        #각속도
        self.angular_velocity = 0.0

        # 각속도를 계산한다. 각속도=(선속도/회전반지름)       
        self.angular_velocity = (self.linear_velocity / self.wheel_base) * np.tan((delta))

        #각변위를 계산해 angle 값에 더해준다. (각속도x시간=각변위)
        self.yaw += (np.degrees(self.angular_velocity) * dt)
        self.yaw = np.degrees(normalize_angle(np.radians(self.yaw)))

        #선속도를 계산한다. (선속도=선형가속도x단위시간)
        self.linear_velocity += (acceleration * dt)
        if game.finish == 1 or game.finish == 2:
                self.linear_velocity = 0
        #삼각비를 이용해 x,y 좌표를 구해준다.
        self.x += (self.linear_velocity * np.cos(np.radians(-self.yaw))) * dt
        self.y += (self.linear_velocity * np.sin(np.radians(-self.yaw))) * dt
        
        #자동차 이미지의 새로운 이미지 좌표를 계산하기 위한 리스트를 선언한다. 
        self.car_x = [0,0,0,0]
        self.car_y = [0,0,0,0]

        #자동차 이미지의 왼쪽상단, 오른쪽상단, 왼쪽하단, 오른쪽하단의 좌표를 이용해서 자동차가 회전한 변위각에 현재 위치를 더하여 자동차의 이동한 위치를 계산한다. 
        for i in range(4):
            self.car_x[i] = self.car_x_ori[i] * np.cos(-radians(self.yaw)) - self.car_y_ori[i] * np.sin(-radians(self.yaw)) + self.x
            self.car_y[i] = self.car_x_ori[i] * np.sin(-radians(self.yaw)) + self.car_y_ori[i] * np.cos(-radians(self.yaw)) + self.y 
        
        #새로운 이미지 좌표 리스트(x, y 각각)에서 가장 작은 값을 반올림한 후 정수로 변환하여 자동차 이미지의 새로운 좌표를 지정한다.
        self.car_img_x = int(round(min(self.car_x)))
        self.car_img_y = int(round(min(self.car_y)))


        #새로 계산된 변위각 만큼 차량 이미지를 회전시킨다.
        self.rotated = pygame.transform.rotate(self.image, self.yaw)
        #회전 시킨 이미지를 새로운 이미지 좌표에 위치하도록 출력한다. 
        self.screen.blit(self.rotated, [self.car_img_x, self.car_img_y])
        # 자동차 중심을 계산
        center_x = sum(self.car_x)/4
        center_y = sum(self.car_y)/4
        self.car_center = [center_x, center_y]
        self.car_front_center = [(self.car_x[2]+self.car_x[3])/2,(self.car_y[2]+self.car_y[3])/2]
        self.car_back_center = [(self.car_x[0]+self.car_x[1])/2,(self.car_y[0]+self.car_y[1])/2]

        self.car_x_list.append(self.car_front_center[0])
        self.car_y_list.append(self.car_front_center[1])
	

    # 정적 메서드, 인스턴스를 만들지 않아도 class의 메서드를 바로 실행가능
    @staticmethod
    def limit_input(delta):
        if delta > C.MAX_STEER:
            return C.MAX_STEER

        if delta < -C.MAX_STEER:
            return -C.MAX_STEER

        return delta

#게임을 실행하는 클래스(main 클래스)
class Game:
    #생성자함수
    def __init__(self):
        #pygame을 초기화 하는 함수
        pygame.init()
        #windows title을 정하는 함수
        pygame.display.set_caption("Car Simulator")
        #pygame window size 설정
        self.screen_width = 1200
        self.screen_height = 850
        #설정된 windows size를 적용하는 함수
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        #while 루프 반복주기. 화면갱신 FPS를 설정하기 위한 시간객체 생성
        self.clock = pygame.time.Clock()
        #while 루프 반복주기
        self.ticks = 60

        #아래 while 루프를 종료시키기 위해 선언하는 변수
        self.exit = False
	
        #자동차 초기 위치
        self.car_position = 5

        self.target_idx = 0
        self.di = 0.0
        self.ai = 0.0
        self.d = 0
        self.re = 1

        # 주차완료를 알리는 변수
        self.finish = 0
        self.angle = 90.0
        self.case =0
    #game을 실행하는 함수
    def run(self):
        #MAP 객체 생성
        mapped = Map(self.screen, self.screen_width, self.screen_height)
        #Car 객체 생성. 처음 진행방향은 위쪽(90도)으로 설정    
        car = Car(300, 700, self.screen, self.angle)

        t, rx, ry, ryaw, v, a, j = [], [], [], [], [], [], []

        while not self.exit:
            car_yaw = 270 - car.yaw	
            #키 이벤트 입력 값 받기
            for event in pygame.event.get():
                if event.type == pygame.MOUSEBUTTONDOWN:
                        # 마우스 x,y 위치를 저장 
                        pos = pygame.mouse.get_pos() 
                        # 마우스 위치에 따라 차량위치 초기화 
                        if pos[0] > 36 and pos[0] < 164 and pos[1] > 118 and pos[1] < 182: 
                                car = Car(100, 150,self.screen, self.angle-90)
                                self.car_position = 1
                                self.d = 0
                        if pos[0] > 36 and pos[0] < 164 and pos[1] > 318 and pos[1] < 414:
                                car = Car(100, 350, self.screen, self.angle-90)
                                self.car_position = 2
                                self.d = 0
                                self.case = 0
                        if pos[0] > 36 and pos[0] < 164 and pos[1] > 518 and pos[1] < 614:
                                car = Car(100, 550, self.screen, self.angle-90)
                                self.car_position = 3	
                                self.d = 0
                                self.case = 0
                        if pos[0] > 36 and pos[0] < 164 and pos[1] > 718 and pos[1] < 814:
                                car = Car(100, 750, self.screen, self.angle-90)
                                self.car_position = 4
                                self.d = 0
                        if pos[0] > 268 and pos[0] < 332 and pos[1] > 636 and pos[1] < 764:
                                car = Car(300, 700, self.screen, self.angle)
                                self.car_position = 5
                                self.d = 0
                        if pos[0] > 468 and pos[0] < 532 and pos[1] > 636 and pos[1] < 764:
                                car = Car(500, 700, self.screen, self.angle)
                                self.car_position = 6
                                self.d = 0
                        if pos[0] > 668 and pos[0] < 732 and pos[1] > 636 and pos[1] < 764:
                                car = Car(700, 700, self.screen, self.angle)
                                self.car_position = 7
                                self.d = 0
                        if pos[0] > 868 and pos[0] < 932 and pos[1] > 636 and pos[1] < 764:
                                car = Car(900, 700, self.screen, self.angle)
                                self.car_position = 8
                                self.d = 0
                        if pos[0] > 1068 and pos[0] < 1132 and pos[1] > 536 and pos[1] < 664:
                                car = Car(1100, 600, self.screen, self.angle)
                                self.car_position = 9
                                self.d = 0
                        if pos[0] > 1068 and pos[0] < 1132 and pos[1] > 236 and pos[1] < 364:
                                car = Car(1100, 300, self.screen, self.angle)
                                self.car_position = 0
                                self.d = 0
                        # 각도버튼을 누를때 차량 포지션 별 이벤트 
                        if self.car_position == 1:
                                if pos[0] > 750 and pos[0] < 814 and pos[1] > 783 and pos[1] < 837:
                                        car = Car(100, 150, self.screen, self.angle-45)		
                                elif pos[0] > 820 and pos[0] < 884 and pos[1] > 783 and pos[1] < 837:
                                        car = Car(100, 150, self.screen, self.angle-90)
                                elif pos[0] > 890 and pos[0] < 954 and pos[1] > 783 and pos[1] < 837:
                                        car = Car(100, 150, self.screen, self.angle-135)
                                elif self.car_position == 2:
                                        if pos[0] > 750 and pos[0] < 814 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(100, 350, self.screen, self.angle-45)
                                        elif pos[0] > 820 and pos[0] < 884 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(100, 350, self.screen, self.angle-90)
                                        elif pos[0] > 890 and pos[0] < 954 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(100, 350, self.screen, self.angle-135)
                                elif self.car_position == 3:
                                        if pos[0] > 750 and pos[0] < 814 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(100, 550, self.screen, self.angle-45)
                                        elif pos[0] > 820 and pos[0] < 884 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(100, 550, self.screen, self.angle-90)
                                        elif pos[0] > 890 and pos[0] < 954 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(100, 550, self.screen, self.angle-135)
                                elif self.car_position == 4:
                                        if pos[0] > 750 and pos[0] < 814 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(100, 750, self.screen, self.angle-45)
                                        elif pos[0] > 820 and pos[0] < 884 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(100, 750, self.screen, self.angle-90)
                                        elif pos[0] > 890 and pos[0] < 954 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(100, 750, self.screen, self.angle-135)
                                elif self.car_position == 5:
                                        if pos[0] > 750 and pos[0] < 814 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(300, 700, self.screen, self.angle+45)
                                        elif pos[0] > 820 and pos[0] < 884 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(300, 700, self.screen, self.angle)
                                        elif pos[0] > 890 and pos[0] < 954 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(300, 700, self.screen, self.angle-45)
                                elif self.car_position == 6:
                                        if pos[0] > 750 and pos[0] < 814 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(500, 700, self.screen, self.angle+45)
                                        elif pos[0] > 820 and pos[0] < 884 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(500, 700, self.screen, self.angle)
                                        elif pos[0] > 890 and pos[0] < 954 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(500, 700, self.screen, self.angle-45)
                                elif self.car_position == 7:
                                        if pos[0] > 750 and pos[0] < 814 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(700, 700, self.screen, self.angle+45)
                                        elif pos[0] > 820 and pos[0] < 884 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(700, 700, self.screen, self.angle)
                                        elif pos[0] > 890 and pos[0] < 954 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(700, 700, self.screen, self.angle-45)
                                elif self.car_position == 8:
                                        if pos[0] > 750 and pos[0] < 814 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(900, 700, self.screen, self.angle+45)
                                        elif pos[0] > 820 and pos[0] < 884 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(900, 700, self.screen, self.angle)
                                        elif pos[0] > 890 and pos[0] < 954 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(900, 700, self.screen, self.angle-45)
                                elif self.car_position == 9:
                                         if pos[0] > 750 and pos[0] < 814 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(1100, 600, self.screen, self.angle+45)
                                         elif pos[0] > 820 and pos[0] < 884 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(1100, 600, self.screen, self.angle)
                                         elif pos[0] > 890 and pos[0] < 954 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(1100, 600, self.screen, self.angle-45)
                                elif self.car_position == 0:
                                         if pos[0] > 750 and pos[0] < 814 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(1100, 300, self.screen, self.angle+45)
                                         elif pos[0] > 820 and pos[0] < 884 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(1100, 300, self.screen, self.angle)
                                         elif pos[0] > 890 and pos[0] < 954 and pos[1] > 783 and pos[1] < 837:
                                                car = Car(1100, 300, self.screen, self.angle-45)
		
		    
                        # planning 버튼을 클릭하면 planning 시작 
                        if pos[0] > 250 and pos[0] < 450 and pos[1] > 783 and pos[1] < 837:
                                t, rx, ry, ryaw, v, a, j = quintic_polynomials_planner(car.car_front_center[0], car.car_front_center[1], car_yaw, 50, 50, 1045, 152, np.deg2rad(315), 30, 50, car.max_acceleration, dt)
                                print("rx : ", rx)
                                print("ry : ", ry)
                        # driving 버튼을 클릭하면 driving 시작 
                        if pos[0] > 480 and pos[0] < 680 and pos[1] > 783 and pos[1] < 837:
                                self.d = 1
                                car.linear_velocity = 50.0

                if event.type == pygame.QUIT:
                    pygame.quit()
	    
	    #단위시간의 크기 설정 - 단위시간이란 1 frame이 지나가는데 걸리는 시간이다.
            #해당 시간이 있어야 속력=거리/시간 등의 공식을 계산할 수 있다.
            dt = float(self.clock.get_time()) / 1000.0

	    #이벤트 감지. 여기선 종료이벤트만 확인하여 루프 종료변수를 True로 변경
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.exit = True

            #windows 화면을 흰색으로 칠한다. 
            self.screen.fill((255, 255, 255))
	# 차량이 AR 태그 부근에 도착하면 주차완료 
            if math.sqrt((1190-car.car_front_center[0])**2+(0-car.car_front_center[1])**2) <95:
                    self.finish = 1	
            #화면 벗어나면 멈춤
            if car.car_front_center[0] < 0 or car.car_front_center[1] < 0:
                    self.finish = 2
	        #변화된 수치를 적용한다.  
                    mapped.update(self.finish)
            #Path Planning해서 생성된 경로 그리기
            if len(rx) != 0:
                    for i, _ in enumerate(rx):
                            pygame.draw.circle(game.screen, SKY_BLUE, [int(rx[i]), int(ry[i])], 3)
	# tracking 결과를 출력 
            if len(car.car_x_list) != 0:
                    for i, _ in enumerate(car.car_x_list):
                            pygame.draw.circle(game.screen, PINK,[int(car.car_x_list[i]), int(car.car_y_list[i])], 3)
            #path tracking
            target_speed = 50
            if self.d !=0:
                    self.target_idx, efa = calc_target_index(car, rx, ry)
                    last_idx1 = len(rx) - 1
                    if last_idx1 > self.target_idx:
                        pygame.draw.circle(game.screen, RED, [int(rx[self.target_idx]), int(ry[self.target_idx])], 3)
                        self.ai = pid_control(target_speed, car.linear_velocity)
                        self.di, self.target_idx = stanley_control(self,car, rx, ry, ryaw, self.target_idx)
                        car.update(self.ai, self.di, dt)		
                    else:
                        if self.re == 1:
                            t2, rx2, ry2, ryaw2, v2, a2, j2 = quintic_polynomials_planner(1045, 152, np.deg2rad(315), 30, 30, 1129, 69, np.deg2rad(315), 20, 20, car.max_acceleration, dt)
                            self.re = 0
		# 추가 path 그리기 
                        for i, _ in enumerate(rx2):
                            pygame.draw.circle(game.screen, ORANGE, [int(rx2[i]), int(ry2[i])], 3)
                        self.target_idx, efa = calc_target_index(car, rx2, ry2)
                        last_idx2 = len(rx2) - 1
                        if last_idx2 > self.target_idx:
                            self.ai = pid_control(target_speed, car.linear_velocity)
                            self.di, self.target_idx = stanley_control(self,car, rx2, ry2, ryaw2, self.target_idx)
                            car.update(self.ai, self.di, dt)
                        else:
                            car.update(10,0,dt)
	    
            if self.d == 0:
                car.linear_velocity = 0
                car.update(self.ai, self.di, dt)

            pygame.display.update()

	    #게임 프레임을 지정 (60fps)
            self.clock.tick(self.ticks)
            
        pygame.quit()
    # 왼쪽과 오른쪽 중 더 짧은 점을 return 해주는 함수
    def distance_btw_twoP(self, p1, p2):
        d1 = (ar_center[0]-p1[0])**2 + (ar_center[1]-p1[1])**2
        d2 = (ar_center[0]-p2[0])**2 + (ar_center[1]-p2[1])**2
        if d1 < d2:
            return "left" 
        elif d2 < d1:
            return "right" 

    def angle_btw(self,p1, p2, p3):
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        deg1 = (360 + degrees(atan2(x2 - x1, y2 - y1))) % 360
        deg2 = (360 + degrees(atan2(x3 - x2, y3 - y2))) % 360
        return -(deg1 - deg2)
#Path Planning 함수
class QuinticPolynomial:
    def __init__(self, xs, vxs, axs, xe, vxe, axe, time):
        #차량 시작 위치
        #길이 3의 배열로 지정된다.
        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        #시간 time에서 초기 상태와 최종 상태를 연결하는 저크 최소화 궤적을 계산한다. 
        A = np.array([[time ** 3, time ** 4, time ** 5],
                      [3 * time ** 2, 4 * time ** 3, 5 * time ** 4],
                      [6 * time, 12 * time ** 2, 20 * time ** 3]])
        b = np.array([xe - self.a0 - self.a1 * time - self.a2 * time ** 2,
                      vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        # 5차 다항식 계산 계수
        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]
    #최소 저크 궤적을 계산한다.
    def calc_point(self, t):
        #최소 저크 궤적은 5차 다항식으로 표시한다.
        #xt는 모든 시간 도함수가 6이고, 저크 최소화를 위해 0이 되어야 한다.
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4 + self.a5 * t ** 5

        return xt

    #3번에 걸쳐서 5차 다항식을 계산한다.
    #위치에 대한 방정식
    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3 + 5 * self.a5 * t ** 4

        return xt

    #속도에 대한 방정식
    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2 + 20 * self.a5 * t ** 3

        return xt

    #가속도에 대한 방정식
    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t ** 2

        return xt

#qauintic polynomials planner 알고리즘을 적용한다.
def quintic_polynomials_planner(sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, max_accel, dt):
    vxs = sv * math.cos(syaw)
    vys = sv * math.sin(syaw)
    vxg = gv * math.cos(gyaw)
    vyg = gv * math.sin(gyaw)

    axs = sa * math.cos(syaw)
    ays = sa * math.sin(syaw)
    axg = ga * math.cos(gyaw)
    ayg = ga * math.sin(gyaw)

    for T in np.arange(MIN_T, MAX_T, MIN_T):
        xqp = QuinticPolynomial(sx, vxs, axs, gx, vxg, axg, T)
        yqp = QuinticPolynomial(sy, vys, ays, gy, vyg, ayg, T)

        time, rx, ry, ryaw, rv, ra, rj = [], [], [], [], [], [], []
        for t in np.arange(0.0, T + dt, dt):
            time.append(t)
            rx.append(xqp.calc_point(t))
            ry.append(yqp.calc_point(t))

            vx = xqp.calc_first_derivative(t)
            vy = yqp.calc_first_derivative(t)
            v = np.hypot(vx, vy)
            yaw = math.atan2(vy, vx)
            rv.append(v)
            ryaw.append(yaw)

            ax = xqp.calc_second_derivative(t)
            ay = yqp.calc_second_derivative(t)
            a = np.hypot(ax, ay)
            if len(rv) >= 2 and rv[-1] - rv[-2] < 0.0:
                a *= -1
            ra.append(a)

            jx = xqp.calc_third_derivative(t)
            jy = yqp.calc_third_derivative(t)
            j = np.hypot(jx, jy)
            if len(ra) >= 2 and ra[-1] - ra[-2] < 0.0:
                j *= -1
            rj.append(j)

        if max([abs(i) for i in ra]) <= max_accel :
            break

    #시간, x, y, yaw, 속도, 엑셀, 저크 결과
    return time, rx, ry, ryaw, rv, ra, rj


def pid_control(target, current):
    pterm = 1.0*(target - current) 
    #iterm += 0.0*(target - current) 
    #dterm = 0.0*(current - prev_current) 
    return pterm

def calc_target_index(car, cx, cy):

    # Calc front axle position
    fx = 84*np.cos(np.deg2rad(-car.yaw))+car.x
    fy = 84*np.sin(np.deg2rad(-car.yaw))+car.y

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(normalize_angle(np.radians(-car.yaw)) + np.pi / 2),
                      -np.sin(normalize_angle(np.radians(-car.yaw)) + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle

def stanley_control(game, car, cx, cy, cyaw, last_target_idx):

    current_target_idx, error_front_axle = calc_target_index(car, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx
    # theta_e corrects the heading error
    theta_e = normalize_angle(-cyaw[current_target_idx] - normalize_angle(np.radians(car.yaw)))
    # theta_d corrects the cross track error
    theta_d = np.arctan2(-0.9 * error_front_axle, car.linear_velocity)
    # Steering control
    delta = theta_e + theta_d

    return delta, current_target_idx
    
def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

if __name__ == '__main__':
    game = Game()
    game.run()
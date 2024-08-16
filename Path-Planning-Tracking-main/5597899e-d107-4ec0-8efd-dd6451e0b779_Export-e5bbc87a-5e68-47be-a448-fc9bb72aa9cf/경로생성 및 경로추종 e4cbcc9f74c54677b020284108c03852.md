# 경로생성 및 경로추종

[https://github.com/AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)

## Path Planning

- Global Path-Planning
    - 출발지부터 목적지까지 전체적인 경로를 계획하는 것
- Local Path-Planning
    - 주변환경을 인지하고 장애물을 회피하는 등의 임시경로를 생성하는 것
    
    ![Untitled](%E1%84%80%E1%85%A7%E1%86%BC%E1%84%85%E1%85%A9%E1%84%89%E1%85%A2%E1%86%BC%E1%84%89%E1%85%A5%E1%86%BC%20%E1%84%86%E1%85%B5%E1%86%BE%20%E1%84%80%E1%85%A7%E1%86%BC%E1%84%85%E1%85%A9%E1%84%8E%E1%85%AE%E1%84%8C%E1%85%A9%E1%86%BC%20e4cbcc9f74c54677b020284108c03852/Untitled.png)
    

- **Quintic Polynomials Planner (5차 다항식을 이용한 경로생성)**
    - X,Y 평면에서 2차원 로봇 동작을 계획하는 5차 다항식 planner
    
- **“jerk”의 비용함수**
    - 비용함수 C는 시간 간격 T 내에서 두 위치 사이를 이동할 때
    - "jerk" 제곱의 시간 적분으로 정의된다.
        
        ![Untitled](%E1%84%80%E1%85%A7%E1%86%BC%E1%84%85%E1%85%A9%E1%84%89%E1%85%A2%E1%86%BC%E1%84%89%E1%85%A5%E1%86%BC%20%E1%84%86%E1%85%B5%E1%86%BE%20%E1%84%80%E1%85%A7%E1%86%BC%E1%84%85%E1%85%A9%E1%84%8E%E1%85%AE%E1%84%8C%E1%85%A9%E1%86%BC%20e4cbcc9f74c54677b020284108c03852/Untitled%201.png)
        

- 비용함수를 최소화하는 결과
    - 2차원 로봇을 위한 5차 다항식
        
        ![Untitled](%E1%84%80%E1%85%A7%E1%86%BC%E1%84%85%E1%85%A9%E1%84%89%E1%85%A2%E1%86%BC%E1%84%89%E1%85%A5%E1%86%BC%20%E1%84%86%E1%85%B5%E1%86%BE%20%E1%84%80%E1%85%A7%E1%86%BC%E1%84%85%E1%85%A9%E1%84%8E%E1%85%AE%E1%84%8C%E1%85%A9%E1%86%BC%20e4cbcc9f74c54677b020284108c03852/Untitled%202.png)
        
    - 5차 다항식의 계산
        
        ```python
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
        ```
        
    

## Path Tracking

- 경로상에 있거나 있지 않은 위치로부터
- 경로를 점근적으로 따라가도록 이동체의 움직임을 제어하는 것
    
    ![Untitled](%E1%84%80%E1%85%A7%E1%86%BC%E1%84%85%E1%85%A9%E1%84%89%E1%85%A2%E1%86%BC%E1%84%89%E1%85%A5%E1%86%BC%20%E1%84%86%E1%85%B5%E1%86%BE%20%E1%84%80%E1%85%A7%E1%86%BC%E1%84%85%E1%85%A9%E1%84%8E%E1%85%AE%E1%84%8C%E1%85%A9%E1%86%BC%20e4cbcc9f74c54677b020284108c03852/Untitled%203.png)
    
- 위치의 오차를 이용하는 것이 아닌 쎄타 값을 이용하여 제어하는 것이 더욱 효과적이고 안정적
- 즉 **Stanley Control** 방식이 더 좋음(**방향 오차** 까지도 고려하기 때문에)
- **Cross Track Error**와 **Heading Error** 를 고려하여 제어하기 때문에 최대 조향 각도를 준수함
    
    ![Untitled](%E1%84%80%E1%85%A7%E1%86%BC%E1%84%85%E1%85%A9%E1%84%89%E1%85%A2%E1%86%BC%E1%84%89%E1%85%A5%E1%86%BC%20%E1%84%86%E1%85%B5%E1%86%BE%20%E1%84%80%E1%85%A7%E1%86%BC%E1%84%85%E1%85%A9%E1%84%8E%E1%85%AE%E1%84%8C%E1%85%A9%E1%86%BC%20e4cbcc9f74c54677b020284108c03852/Untitled%204.png)
    
    - **Cross Track Error** : 계획된 트랙으로부터의 직선거리
    - **Heading Error** : 트랙 진행 방향과 차량 진행 방향의 차이
    
    ![Untitled](%E1%84%80%E1%85%A7%E1%86%BC%E1%84%85%E1%85%A9%E1%84%89%E1%85%A2%E1%86%BC%E1%84%89%E1%85%A5%E1%86%BC%20%E1%84%86%E1%85%B5%E1%86%BE%20%E1%84%80%E1%85%A7%E1%86%BC%E1%84%85%E1%85%A9%E1%84%8E%E1%85%AE%E1%84%8C%E1%85%A9%E1%86%BC%20e4cbcc9f74c54677b020284108c03852/Untitled%205.png)
    
    - 위 그림에서는 Heading error에 대한 설명으로 보이는데, Cross Track Error의 내용이 나와서 헷갈린다.

- 아래는 Stanley Control을 구현한 예제 코드이다
    
    ```python
    def stanley_control(state, cx, cy, cyaw, last_target_idx):
        """
        Stanley steering control.
    
        :param state: (State object)
        :param cx: ([float])
        :param cy: ([float])
        :param cyaw: ([float])
        :param last_target_idx: (int)
        :return: (float, int)
        """
        current_target_idx, error_front_axle = calc_target_index(state, cx, cy)
    
        if last_target_idx >= current_target_idx:
            current_target_idx = last_target_idx
    
        # theta_e corrects the heading error
        theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
        # theta_d corrects the cross track error
        theta_d = np.arctan2(k * error_front_axle, state.v)
        # Steering control
        delta = theta_e + theta_d
    
        return delta, current_target_idx
    
    def normalize_angle(angle):
        """
        Normalize an angle to [-pi, pi].
    
        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi
    
        while angle < -np.pi:
            angle += 2.0 * np.pi
    
        return angle
    
    def calc_target_index(state, cx, cy):
        """
        Compute index in the trajectory list of the target.
    
        :param state: (State object)
        :param cx: [float]
        :param cy: [float]
        :return: (int, float)
        """
        # Calc front axle position
        fx = state.x + L * np.cos(state.yaw)
        fy = state.y + L * np.sin(state.yaw)
    
        # Search nearest point index
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)
    
        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                          -np.sin(state.yaw + np.pi / 2)]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)
    
        return target_idx, error_front_axle
    ```
    

- **느낀 점**
    - Plannin을 할 때는 quintic_polynomials_planner 방식을 이용하는 것이 좋아보인다. 또한 tracking시에는 stanley_control방식을 쓰는 것이 좋아 보인다. 이를 위해 pythonrobotics의 자료들을 참고하여 진행했는데, 과제에서의 조건들이 어긋나 보이는 경우가 종종 보였다. 기존에 진행했던 국민대 대회 참가 팀에서는 pure_pursuit 방식 또한 사용한것으로 보인다. 최단 경로 알고리즘같은 것들은 많이 해봤는데, 차량의 특성과 곡선 표현을 신경써야 하다 보니 그래프에서의 최단 경로 구하는 것과는 상당 부분 차이가 있다. 다른 팀의 코드를 보고 공부하는 것이 좋을 듯.
    - **Path Planning : Polynomial**
    - **Path Tracking : Pure-Pursuit**
        
        ```python
        #!/usr/bin/env python
        #-- coding:utf-8 --
        
        #=============================================
        # 함께 사용되는 각종 파이썬 패키지들의 import 선언부
        #=============================================
        import pygame
        import numpy as np
        import math
        import rospy
        import rospkg
        from xycar_msgs.msg import xycar_motor
        import time
        #=============================================
        # 모터 토픽을 발행할 것임을 선언
        #============================================= 
        motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        xycar_msg = xycar_motor()
        
        #=============================================
        # 프로그램에서 사용할 변수, 저장공간 선언부
        #============================================= 
        rx, ry = [300, 350, 400, 450], [300, 350, 400, 450]
        CHANGE = False #방향이 전환되는 여부: 후진에서 전진으로 바뀌는 경우 CHANGE = True
        
        #=============================================
        # 프로그램에서 사용할 상수 선언부
        #=============================================
        AR = (1142, 62) # AR 태그의 위치
        P_ENTRY = (1036, 162) # 주차라인 진입 시점의 좌표
        
        P_END = (1129, 69) # 주차라인 끝의 좌표
        GAP = 30 # 도착지점 앞부분 길이 설정에 필요한 gap
        
        X_MAX = 1036 # 도착지점 앞부분의 마지막 x 좌표
        X_MIN = X_MAX - GAP # 도착지점 앞부분의 처음 x좌표
        
        Y_MIN = 162 #도착지점 앞부분의 처음 y 좌표
        Y_MAX = Y_MIN +GAP # 도착지점 앞부분의 마지막 y 좌표
        
        HALF_CAR = 64 # 자동차의 절반 길이
        #=============================================
        # 모터 토픽을 발행하는 함수
        # 입력으로 받은 angle과 speed 값을
        # 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
        #=============================================
        def drive(angle, speed):
            xycar_msg.angle = int(angle)
            xycar_msg.speed = int(speed)
            motor_pub.publish(xycar_msg)
        
        #차량의 머리 좌표 계산 함수
        def head_point(x, y, yaw):
            x = x + HALF_CAR * np.cos(np.radians(yaw))
            y = y - HALF_CAR * np.sin(np.radians(yaw))
            return x, y
        
        #차량의 꼬리 좌표 계산 함수
        def back_point(x, y, yaw):
            x = x - HALF_CAR * np.cos(np.radians(yaw))
            y = y + HALF_CAR * np.sin(np.radians(yaw))
            return x, y
        
        #=============================================
        # 주차 구역과의 거리가 먼 경우 경로의 궤적을 생성하는 함수
        # 현재 좌표값과 deg(차수) 값을 받아 주차구역으로 향하는
        # deg 차 함수로 이루어진 궤적 형성
        #=============================================
        def get_approximate_traj(sx, sy, deg):
            ''' [Tool] 근사 궤적을 구하는 함수 '''
            # 도착 지점입구 부분에 기울기가 1인 포인트 리스트 생성
            x = list(range(X_MAX, X_MIN, -1))
            y = list(range(Y_MIN, Y_MAX, 1))
            
            #현재 좌표 리스트화
            start_x = [sx]
            start_y = [sy]
        
            #현재 좌표와 생성한 도착 지점 리스트 결합
            x_point = start_x + x
            y_point = start_y + y
        
            #결합한 포인트들을 가지고 deg차수의 함수 생성
            args = np.polyfit(x_point, y_point, deg)
            func = np.poly1d(args)
        
            #현재 좌표에서부터 도착 지점 부근 경로까지 리스트 생성 후 return
            if sx < X_MIN:
                X = np.arange(sx, X_MIN)
                Y = func(X)
        
            else:
                X = np.arange(X_MIN,sx)
                Y = func(X)
        
            return X, Y
        
        #=============================================
        # 주차 구역과의 거리가 가까운 경우 경로의 궤적을 생성하는 함수
        # 현재 좌표값과 deg(차수) 값을 받아 주차구역으로 향하는
        # deg 차 함수로 이루어진 궤적 형성
        #=============================================
        def get_approximate_traj2(sx, sy, deg):
            ''' [Tool] 근사 궤적을 구하는 함수 '''
        
            #추자 구역에서 일정거리 떨거진 점의 좌표 리스트화
            x = [X_MIN-300]
            y = [Y_MAX+300]
            
            #현재 좌표 리스트화
            start_x = [sx]
            start_y = [sy]
        
            #현재 좌표와 생성한 도착 지점 리스트 결합
            x_point = start_x + x
            y_point = start_y + y
        
            #결합한 포인트들을 가지고 deg차수의 함수 생성
            args = np.polyfit(x_point,y_point, deg)
            func = np.poly1d(args)
        
            #현재 좌표에서부터 주차 구역에서 일정거리 떨어진 점까지 리스트 생성 후 return
            if sx < X_MIN-300:
                X = np.arange(X_MIN-300, sx, -1)
                Y = func(X)
        
            else:
                X = np.arange(sx, X_MIN-300, -1)
                Y = func(X)
        
            return X, Y
        
        #=============================================
        # 경로를 생성하는 함수
        # 차량의 시작위치 sx, sy, 시작각도 syaw
        # 최대가속도 max_acceleration, 단위시간 dt 를 전달받고
        # 경로를 리스트를 생성하여 반환한다.
        #=============================================
        def planning(sx, sy, syaw, max_acceleration, dt):
            global rx, ry, CHANGE #경로 좌표, 방향전환 여부
            print("Start Planning")
        
            CHANGE = False #hit_point_num 초기화
        
            #시작 순간의 현재 좌표를 차량의 head point로 전환
            sx = sx - HALF_CAR * np.sin(np.radians(syaw))
            sy = sy + HALF_CAR * np.cos(np.radians(syaw))
        
            #경로 생성부분
            if (X_MIN - sx) >30: #주차 구역과 시작점의 거리가 먼 경우
                rx, ry = get_approximate_traj(sx, sy, 2) # 출발지점부터 주차 지점까지 2차 궤적 형성 후 경로로 저장
                x = list(range(X_MIN, 1129 ,1)) # 도착 지점 일정구간 앞에서부터는 직진구간 (x좌표)
                y = list(range(Y_MAX,69, -1)) # (y좌표)
        
                #전체 경로 합성
                rx = list(rx) + x 
                ry = list(ry) + y
        
            else: #주차 구역과 시작점의 거리가 가까운 경우
                rx, ry = get_approximate_traj2(sx, sy, 2) #궤적 생성
                x = list(range(X_MIN-300, 1129 ,1)) # 도착 지점에서 추자 구역에서 일정거리 떨어진 점까지 직진구간 (x좌표)
                y = list(range(Y_MAX+300, 69, -1)) # (y좌표)
        
                #전체 경로 합성
                rx = list(rx) + x 
                ry = list(ry) + y
        
            return rx, ry
        
        #=============================================
        # 생성된 경로를 따라가는 함수
        # 파이게임 screen, 현재위치 x,y 현재각도, yaw
        # 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
        # 각도와 속도를 결정하여 주행한다.
        #=============================================
        
        def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
            global rx, ry, CHANGE
        
            is_look_forward_point = False #lfd 비활성
            x_h, y_h = head_point(x, y, yaw) #차량의 head_point 좌표
            x, y = back_point(x, y, yaw) #차량의 back_point 좌표
        
            #속도, 조향각 초기화
            speed = 50
            angle = 0
        
            FORWARD = True #전진 활성화(초기화)
            lfd = 200 #look forward distance: purepursuit 알고리즘에서 전방주시거리
            L = 2 * HALF_CAR #차량의 길이
            
            ######################################
            #           purepursuit              #
            ######################################
            for i in range(len(rx)-1): #path 포인트 순회
                #현재 순회하는 포인트
                path_point_x = rx[i]
                path_point_y = ry[i]
        
                next_point_x = rx[i + 1] #다음 포인트의 x 좌표
                point_x_gap = next_point_x - path_point_x #현재 순회하는 포인트와 다음 포인트의 x 좌표 차이
                
                #차량의 후방 좌표와 경로 포인트의 x, y 좌표 차이 
                dx = path_point_x - x
                dy = path_point_y - y
        
                #purepursuit 알고리즘의 조향각 계산을 위한 수치 계산
                arpha = math.atan2(dy, dx) 
                yaw_radian = math.radians(yaw)
                theta = (math.pi/2) - (arpha + yaw_radian)
        
                rotated_x = math.sin(theta) * lfd #자동차기준 전방을 x 축으로 하였을때 포인트와의 x거리 (차량 전방기준 포인트 위치)
                dis = math.sqrt(pow(dx, 2) + pow(dy, 2)) #차량의 후방에서부터 path 까지의 거리
                
                #lfd 선정하는 부분
                if CHANGE is False: #방향전환 되지 않았을 때
                    if point_x_gap < 0: #point 의 x 좌표가 줄어드는 경우
                        FORWARD = False #후진
        
                        if rotated_x < 0: #차량보다 뒤에 있는 점들만 순회
                            if dis >= lfd : #dis 가 lfd 보다 큰 경우 break
                                is_look_forward_point = True #lfd 활성화
                                break
        
                    else: #point 의 x 좌표가 증가하는 경우
                        if abs(math.sin(theta) * dis) <= 5: #차량의 후방과 point의 거리가 5 미만이 됐을때 전진으로 변경
                            FORWARD = True #전진
        
                        if FORWARD == False: #후진하는 동안 차량이 포인트에 접근하기 전까지는 포인트 유지
                            break
        
                        if rotated_x > 0: #차량보다 앞에 있는 점들만 순회
                            if dis >= lfd : #dis 가 lfd 보다 큰 경우 break
                                is_look_forward_point = True #lfd 활성화
                                CHANGE = True #방향전환 활성화
                                break
        
                else: #방향전환 되었을 때
                    if point_x_gap > 0: #point 의 x 좌표가 증가하는 경우
                        if rotated_x > 0: #차량보다 앞에 있는 점들만 순회
                            if dis >= lfd : #dis 가 lfd 보다 큰 경우 break
                                is_look_forward_point = True #lfd 활성화
                                break
        
            pygame.draw.line(screen, (255,0,0), (x+np.cos(arpha)*lfd, y+np.sin(arpha)*lfd), (x,y), 1) #차량에서 point 까지 연결하는 직선 출력(빨간선)
            pygame.draw.line(screen, (0,0,255), (x+dx, y), (x,y), 1) #차량에서 부터 dx 표시(파란선)
            pygame.draw.line(screen, (0,0,255), (x+dx, y), (x+dx,y+dy), 1) #차량에서부터 dy 표시 (파란선)
        
            distance = math.sqrt(pow(1129 - x_h, 2) + pow(69 - y_h , 2)) #차량의 head에서부터 주차지점의 end 값까지 거리
        
            if is_look_forward_point : #lfd 가 활성화 되었을때
                steering=math.atan2((4* math.cos(theta)*L),lfd)#조향각 계산
                angle = int(math.degrees(steering)) #조향각을 angle로 할당
        
                #조향각이 50보다 큰경우 50으로 고정
                if angle >= 50:
                    angle = 50
        
                #조향각이 -50보다 작은경우 -50으로 고정
                elif angle <= -50:
                    angle = -50
        
                #후진이 활성화 돼있으면 speed -50
                if FORWARD == False:
                    speed = -50
        
                drive(angle, speed)
        
            else : #목표지점과 거리가 3이하가 되었을때 정지
                if distance <= 3:
                    angle = 0
                    speed = 0
            
                    drive(angle, speed)
            
            beta = math.radians(-angle+yaw) #조향각을 차량 기준으로 전환
        
            pygame.draw.line(screen, (0,255,0), (x_h+np.cos(beta)*lfd, y_h-np.sin(beta)*lfd), (x_h,y_h), 3) #차량 기준으로 조향각 출력 (초록선)
            pygame.draw.circle(screen, (255,0,0), (x, y), lfd, 1) #차량 기준으로 lfd 범위 출력 (빨간 원)
            pygame.draw.circle(screen, (0,0,255), (path_point_x, path_point_y), 5, 5) #목표지점 (lfd 포인트) 표시(파란점)
            pygame.draw.circle(screen, (0,0,255), (x, y), 10, 10) #차량후방에 점 출력(파란 점)
        ```
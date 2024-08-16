#! /usr/bin/env python

# 2023-10-06 목요일
# Created by 유준호
# AR tag를 인식하여 Path Planning 및 Path Tracking을 수행하는 프로그램
# Path Planning Algorithm : Quintic-Polynomial Planner
# Path Tracking Algorithm : Pure-Pursuit Controller

import pygame, sys, math, rospy
import numpy as np
import matplotlib.pyplot as plt

from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from xycar_msgs.msg import xycar_motor

# 파이게임 초기화
pygame.init()

# 2m     -> 800pixel
# 1cm    -> 4pixel
# 0.25cm -> 1pixel

# 화면 크기 설정
SCREEN_SCALE = 2 # zoom in or zoom out
SCREEN_WIDTH , SCREEN_HEIGHT = 400*SCREEN_SCALE , 400*SCREEN_SCALE # 4m

#  total length of Xycar is 45cm
CAR_WIDTH, CAR_HEIGHT = 20*SCREEN_SCALE, 45*SCREEN_SCALE # 20cm, 45cm
car_wheel_yaw_degree = 90

CAR_WHEEL_LENGTH = 9*SCREEN_SCALE # 9cm
CAR_FROM_REAR_TO_CAMERA_LENGTH = int((CAR_HEIGHT*2)/3) # 30cm
CAR_FROM_CAMERA_TO_HEAD_LENGTH = int(CAR_HEIGHT/3) # 15cm

# 30cm -> length from tail to camera
SCREEN_BLANK_HEIGHT = int(CAR_FROM_REAR_TO_CAMERA_LENGTH*1.2 )

screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT+SCREEN_BLANK_HEIGHT))
pygame.display.set_caption("juno_ar_path_sim")

# 색상 정의
BLACK = (0, 0, 0)
RED = (255,0,0)
BLUE = (0, 0, 255)
WHITE = (255, 255, 255)
GREY = (211,211,211)
DARK_GREY = (100,100,100)
DARK_RED = (128,0,0)
PINK   = (255, 0  , 127)
GREEN  = (0  , 255, 0)
SKY    = (0  , 200, 200)
YELLOW = (200, 200, 0)

# 텍스트 폰트 설정
font_ar = pygame.font.SysFont("arial", 15, True, True)
font_steering = pygame.font.SysFont("arial", 20, True, True)

CAR_CAMERA_X, CAR_CAMERA_Z = SCREEN_WIDTH/2 , SCREEN_HEIGHT # 시뮬레이터 상에서 차량의 위치는 고정되어 있음
CAR_CENTER_X, CAR_CENTER_Z = SCREEN_WIDTH/2,  SCREEN_HEIGHT - CAR_FROM_CAMERA_TO_HEAD_LENGTH + CAR_HEIGHT/2 
CAR_LEFT_TOP_X, CAR_LEFT_TOP_Z= SCREEN_WIDTH/2-CAR_WIDTH/2, SCREEN_HEIGHT-CAR_FROM_CAMERA_TO_HEAD_LENGTH # for Rect()

CAR_CENTER_LENGTH = CAR_HEIGHT*0.6
CAR_CENTER_START_X = CAR_CENTER_X
CAR_CENTER_START_Z = CAR_CENTER_Z-CAR_CENTER_LENGTH/2
CAR_CENTER_END_X, CAR_CENTER_END_Z = CAR_CENTER_X, CAR_CENTER_START_Z+CAR_CENTER_LENGTH

CAR_WHEEL_AXIS_LENGTH = CAR_WIDTH*0.5
CAR_FRONT_WHEEL_START_X , CAR_FRONT_START_Z = CAR_CENTER_X-CAR_WHEEL_AXIS_LENGTH/2, CAR_CENTER_Z-CAR_CENTER_LENGTH/2
CAR_FRONT_WHEEL_END_X   , CAR_FRONT_END_Z   = CAR_CENTER_X+CAR_WHEEL_AXIS_LENGTH/2, CAR_FRONT_START_Z
CAR_FRONT_WHEEL_CENTER_X, CAR_FRONT_WHEEL_CENTER_Z = int(CAR_CENTER_X), int(CAR_CENTER_Z - CAR_CENTER_LENGTH/2)

CAR_REAR_WHEEL_START_X , CAR_REAR_START_Z = CAR_FRONT_WHEEL_START_X, CAR_CENTER_Z+CAR_CENTER_LENGTH/2
CAR_REAR_WHEEL_END_X   , CAR_REAR_END_Z   = CAR_FRONT_WHEEL_END_X, CAR_REAR_START_Z
CAR_REAR_WHEEL_CENTER_X, CAR_REAR_WHEEL_CENTER_Z = int(CAR_CENTER_X), int(CAR_CENTER_Z + CAR_CENTER_LENGTH/2)


moved_car_pos = (CAR_CAMERA_X, CAR_CAMERA_Z) # AR의 위치 변화에 따른 가상의 차량을 위한 위치 정보


parking_slot_entrance_x = 0
parking_slot_entrance_z = 0
SCALE_PARKING_SLOT = 1.3
LENGTH_FROM_AR_TO_SLOT = 40

# 버튼 생성을 위한 정보
BUTTON_HOVER_COLOR = (0, 200, 255)
font_button = pygame.font.SysFont("arial", 20, True, True)
BUTTON_WIDTH, BUTTON_HEIGHT = 200, 50

BUTTON_X_PATH_PLANNING, BUTTON_Z_PATH_PLANNING = 0,0
BUTTON_COLOR_PATH_PLANNING = (0, 128, 255)
BUTTON_TEXT_PATH_PLANNING = font_button.render("Path Planning", True, (255, 255, 255))

BUTTON_X_PATH_TRACKING, BUTTON_Z_PATH_TRACKING = BUTTON_WIDTH, 0
BUTTON_COLOR_PATH_TRACKING = (0, 0, 128)
BUTTON_TEXT_PATH_TRACKING = font_button.render("Path Tracking", True, (255, 255, 255))

BUTTON_X_RESET, BUTTON_Z_RESET = BUTTON_WIDTH*2, 0
BUTTON_COLOR_RESET = (128, 0, 128)
BUTTON_TEXT_RESET = font_button.render("RESET", True, (255, 255, 255))



# ar tag의 위치 정보
AR_LENGTH = 20*SCREEN_SCALE  # 10cm
edited_ar_yaw_rad = 0
ar_delta_x, ar_delta_z = 0.0, 0.0

motor_msg = xycar_motor()

path_planned_x, path_planned_z = [],[]

ar_clicked_x, ar_clicked_z, ar_clicked_yaw_rad = 0.0, 0.0, 0.0

arData= {
        "DX":0.0, 
        "DY":0.0, 
        "DZ":0.0, 
        "AX":0.0, 
        "AY":0.0, 
        "AZ":0.0, 
        "AW":0.0
        }

flag_ar_detected=False
flag_button_planning_clicked=False
flag_button_tracking_clicked=False
flag_button_reset_clicked=False
flag_show_path=False
flag_arrival = False

# CONTROL PARAMETERS
heading_error = 0 # used in Stanley Controller, not in Pure-Pursuit
LOOK_AHEAD_DISTANCE = 200 # 50cm
look_ahead_point_x, look_ahead_point_z = 0,0
look_ahead_point_dx, look_ahead_point_dz = 0 , 0

yaw_degrees = 0 # Yaw 값을 담아두기 위한 전역변수

# ar_track_alvar로부터 날아오는 ar토픽을 받았을 때 실행되는 콜백 함수이다.
# 값의 저장 및 최초로 넘어오는 순간을 알기 위해 flag_ar_detected 변수를 사용한다.
def callback(msg):
    global flag_ar_detected
    global arData
    
    for i in msg.markers:
        arData["DX"] = i.pose.pose.position.x
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z

        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w

    if arData["DX"]!=0.0 and arData["DY"]!=0.0 and arData["DZ"]!=0.0:
        flag_ar_detected = True

# 흰 배경과 격자 무늬를 넣는다.
def draw_background():

    # Background
    screen.fill(WHITE)
    grid_num = 10
    for i in range(0,SCREEN_HEIGHT+1, int(SCREEN_WIDTH/grid_num)):
        pygame.draw.line(screen, GREY, (0,i),(SCREEN_WIDTH,i),3)
    for i in range(0,SCREEN_WIDTH+1, int(SCREEN_WIDTH/grid_num)):
        pygame.draw.line(screen, GREY, (i,0),(i, SCREEN_HEIGHT),3)

    # Path Planning과 Tracking을 위한 버튼을 그려주는 함수
    draw_button_planning()
    draw_button_tracking()
    draw_button_reset()

# 바퀴를 그려주는 함수. 각도의 조향을 위해 분리시켰다.
def draw_car_wheel(axis_x , axis_z , car_yaw_radians):
    car_end_pos = (
        (
            axis_x + CAR_WHEEL_LENGTH/2 * math.cos(car_yaw_radians),
            axis_z - CAR_WHEEL_LENGTH/2 * math.sin(car_yaw_radians)
        ),
        (
            axis_x - CAR_WHEEL_LENGTH/2 * math.cos(car_yaw_radians),
            axis_z + CAR_WHEEL_LENGTH/2 * math.sin(car_yaw_radians)
        )
    )

    # 바퀴의 회전을 표현하기 위해 원의 중심을 기준으로 서로 반대로 회전하는 선 두 개를 그려줌.
    pygame.draw.line(screen, BLACK, (axis_x, axis_z), car_end_pos[0], 6)
    pygame.draw.line(screen, BLACK, (axis_x, axis_z), car_end_pos[1], 6)

# 차량과 구동축들을 그려주는 함수
def draw_car_body():
    pygame.draw.rect(screen, BLACK, pygame.Rect(CAR_LEFT_TOP_X, CAR_LEFT_TOP_Z, CAR_WIDTH, CAR_HEIGHT), 2)
    pygame.draw.line(screen, BLACK, (CAR_CENTER_START_X, CAR_CENTER_START_Z), 
                                    (CAR_CENTER_END_X, CAR_CENTER_END_Z), 2)
    pygame.draw.line(screen, BLACK, (CAR_FRONT_WHEEL_START_X, CAR_FRONT_START_Z), 
                                    (CAR_FRONT_WHEEL_END_X, CAR_FRONT_END_Z), 2)
    pygame.draw.line(screen, BLACK, (CAR_REAR_WHEEL_START_X, CAR_REAR_START_Z), 
                                    (CAR_REAR_WHEEL_END_X, CAR_REAR_END_Z), 2)


def draw_look_ahead_area():
    pygame.draw.circle(screen, BLACK, (CAR_REAR_WHEEL_CENTER_X, CAR_REAR_WHEEL_CENTER_Z), LOOK_AHEAD_DISTANCE, 3)


# 차량을 그리는 함수 
def draw_car(angle_radians):

    draw_look_ahead_area()

    draw_car_body() # 고정된 차체의 부분들

    # x, z, yaw(degrees)
    # 앞 바퀴는 좌,우로 회전하기 때문에 따로 각도를 주었다. 뒷 바퀴는 예외다.
    draw_car_wheel(CAR_FRONT_WHEEL_START_X, CAR_FRONT_START_Z, angle_radians) # 왼쪽 앞 바퀴
    draw_car_wheel(CAR_FRONT_WHEEL_END_X, CAR_FRONT_END_Z, angle_radians - math.pi) # 오른쪽 앞 바퀴
    draw_car_wheel(CAR_REAR_WHEEL_START_X, CAR_REAR_START_Z, -math.pi/2) # 왼쪽 뒷 바퀴
    draw_car_wheel(CAR_REAR_WHEEL_END_X, CAR_REAR_END_Z, -math.pi/2) # 오른쪽 뒷 바퀴













# AR tag와 동일하게 움직이는 주차 구역 그리는 함수
# Path-Planning을 위한 주차장의 입구 좌표를 반환한다.
def draw_ar_parking_slot(ar_x, ar_z):
    global parking_slot_entrance_x, parking_slot_entrance_z, edited_ar_yaw_rad

    ar_yaw_radians_minus_90_degree = edited_ar_yaw_rad-math.pi/2
    
    SLOT_X = ar_x - LENGTH_FROM_AR_TO_SLOT*math.cos(ar_yaw_radians_minus_90_degree)
    SLOT_Z = ar_z - LENGTH_FROM_AR_TO_SLOT*math.sin(ar_yaw_radians_minus_90_degree)

    slot_point0 = (
        SLOT_X - (CAR_WIDTH*(SCALE_PARKING_SLOT+0.2))/2 * math.cos(edited_ar_yaw_rad),
        SLOT_Z - (CAR_WIDTH*(SCALE_PARKING_SLOT+0.2))/2 * math.sin(edited_ar_yaw_rad)
    )
    slot_point1 = (
        SLOT_X + (CAR_WIDTH*(SCALE_PARKING_SLOT+0.2))/2 * math.cos(edited_ar_yaw_rad),
        SLOT_Z + (CAR_WIDTH*(SCALE_PARKING_SLOT+0.2))/2 * math.sin(edited_ar_yaw_rad)
    )
    
    # 회전된 선 그리기
    pygame.draw.line(screen, BLACK, (SLOT_X, SLOT_Z), slot_point0, 5)
    pygame.draw.line(screen, BLACK, (SLOT_X, SLOT_Z), slot_point1, 5)

    slot_point2 = (
        slot_point0[0] - (CAR_HEIGHT*SCALE_PARKING_SLOT) * math.cos(ar_yaw_radians_minus_90_degree),
        slot_point0[1] - (CAR_HEIGHT*SCALE_PARKING_SLOT) * math.sin(ar_yaw_radians_minus_90_degree)
    )
    slot_point3 = (
        slot_point1[0] - (CAR_HEIGHT*SCALE_PARKING_SLOT) * math.cos(ar_yaw_radians_minus_90_degree),
        slot_point1[1] - (CAR_HEIGHT*SCALE_PARKING_SLOT) * math.sin(ar_yaw_radians_minus_90_degree)
    )

    # 회전된 선 그리기
    pygame.draw.line(screen, BLACK, slot_point2, slot_point0, 5)
    pygame.draw.line(screen, BLACK, slot_point3, slot_point1, 5)
    pygame.draw.line(screen, BLACK, slot_point2, slot_point3, 5)
    
    # 
    parking_slot_entrance_x = (slot_point2[0] + slot_point3[0])/2
    parking_slot_entrance_z = (slot_point2[1] + slot_point3[1])/2

    return ( (slot_point2[0] + slot_point3[0])/2 ,  (slot_point2[1] + slot_point3[1])/2 )


# AR tag의 위치정보를 받아와 이를 중심으로 직선 두 개를 긋는다. 
# 중심 점의 yaw값에 따라 이 두 선은 회전한다.
def draw_ar(ar_x, ar_z):
    global edited_ar_yaw_rad    

    # draw_lines()
    pygame.draw.line(screen, PINK , (CAR_CAMERA_X, CAR_CAMERA_Z), (ar_x, ar_z), 2)
    pygame.draw.line(screen, BLUE, (CAR_CAMERA_X, CAR_CAMERA_Z), (CAR_CAMERA_X, ar_z), 2)
    pygame.draw.line(screen, BLUE, (CAR_CAMERA_X, ar_z_pos) , (CAR_CAMERA_X + (ar_x-CAR_CAMERA_X) * 2, ar_z), 2)
    
    right_angle_of_theta = arctan_angle_cam2ar_rad
    
    # 대각선의 수선을 표현하기 위한 위치 정보
    perpendicular_end_pos = (
        (
            ar_x - AR_LENGTH*4 * math.cos(right_angle_of_theta),
            ar_z - AR_LENGTH*4 * math.sin(right_angle_of_theta)
        ),
        (
            ar_x - AR_LENGTH*4 * math.cos(right_angle_of_theta + math.pi),
            ar_z - AR_LENGTH*4 * math.sin(right_angle_of_theta + math.pi)
        )
    )
    pygame.draw.line(screen, RED, perpendicular_end_pos[0], (ar_x, ar_z), 2)
    pygame.draw.line(screen, RED, perpendicular_end_pos[1], (ar_x, ar_z), 2)

    # AR tag의 회전 표현을 위한 위치 정보 생성
    ar_end_pos = (
        (
            ar_x - AR_LENGTH/2 * math.cos(edited_ar_yaw_rad),
            ar_z - AR_LENGTH/2 * math.sin(edited_ar_yaw_rad)
        ),
        (
            ar_x + AR_LENGTH/2 * math.cos(edited_ar_yaw_rad),
            ar_z + AR_LENGTH/2 * math.sin(edited_ar_yaw_rad)
        )
    )
    # 선의 끝점 계산
    ar_end_beyond_pos = (
        (
            ar_x - AR_LENGTH*4 * math.cos(edited_ar_yaw_rad),
            ar_z - AR_LENGTH*4 * math.sin(edited_ar_yaw_rad)
        ),
        (
            ar_x + AR_LENGTH*4 * math.cos(edited_ar_yaw_rad),
            ar_z + AR_LENGTH*4 * math.sin(edited_ar_yaw_rad)
        )
    )
    # AR tag의 위치정보를 중심으로 회전된 두 선 그리기
    pygame.draw.line(screen, BLACK, (ar_x, ar_z), ar_end_beyond_pos[0], 2)
    pygame.draw.line(screen, BLACK, (ar_x, ar_z), ar_end_beyond_pos[1], 2)
    pygame.draw.line(screen, DARK_GREY, (ar_x, ar_z), ar_end_pos[0], 8)
    pygame.draw.line(screen, DARK_GREY, (ar_x, ar_z), ar_end_pos[1], 8)

    
    # 위치, 회전 정보 화면 출력
    ar_text = font_ar.render("[z : "+str(int(arData["DZ"]*100))+"cm]   " + \
                             "[x : " + str(int(arData["DX"]*100)) + "cm]   " + \
                             "[yaw : "+str(int(yaw_degrees))+"']   " + \
                             "[theta : " + str(int(math.degrees(arctan_angle_cam2ar_rad))) + "']",
                             True, (0,0,255)) 
    text_rect = ar_text.get_rect()
    text_rect.center = (ar_x,ar_end_pos[0][1]-30)
    screen.blit(ar_text, text_rect)
    
    # AR tag와 동일하게 움직이는 주차 구역 그리는 함수
    draw_ar_parking_slot(ar_x, ar_z)











def calc_dist(pos1, pos2):
    global look_ahead_point_dx, look_ahead_point_dz
    look_ahead_point_dx = pos1[0] - pos2[0]
    look_ahead_point_dz = pos1[1] - pos2[1]

    return int(math.sqrt(look_ahead_point_dx**2 + look_ahead_point_dz**2))



# 2차원 상의 경로를 점의 형태로 그려준다.
def draw_path(path_x, path_z):
    # 직선구간을 임의로 만들어주어 직진하도록 유도함
    linear_function = create_linear_function((parking_slot_entrance_x, parking_slot_entrance_z) , (ar_x_pos, ar_z_pos))
    x_start = min(ar_x_pos, parking_slot_entrance_x)
    x_end = max(ar_x_pos, parking_slot_entrance_x)
    x = x_start
    dx_temp = x_end - x_start
    num_points = 100
    for i in range(num_points):
        path_x.append(x)
        path_z.append(linear_function(x))
        x = x_start + dx_temp*i/num_points


    ################################################################
    # Look-Ahead Distance에 있는 점을 발견하여 그 위치정보를 저장함
    ################################################################
    global look_ahead_point_x,look_ahead_point_z
    flag_lfc_detected = False
    for i in range(len(path_x)):
        if flag_lfc_detected == False:
            pygame.draw.circle(screen, RED, (int(path_x[i]),int(path_z[i])), 2)

            if calc_dist((CAR_REAR_WHEEL_CENTER_X, CAR_REAR_WHEEL_CENTER_Z), (path_x[i],path_z[i])) >= LOOK_AHEAD_DISTANCE:
                if path_z[i] < CAR_REAR_WHEEL_CENTER_Z:
                    look_ahead_point_x, look_ahead_point_z = int(path_x[i]),int(path_z[i])
                    pygame.draw.circle(screen, BLUE, (look_ahead_point_x,look_ahead_point_z), 7)   
                    flag_lfc_detected = True 
        else:
            pygame.draw.circle(screen, RED, (int(path_x[i]),int(path_z[i])), 2)

    if flag_lfc_detected == False:
        look_ahead_point_x, look_ahead_point_z = parking_slot_entrance_x, parking_slot_entrance_z
    

################################################################
# QuinticPolynomial Path Planning을 위한 클래스 선언
################################################################
class QuinticPolynomial:

    def __init__(self, xs, vxs, axs, xe, vxe, axe, time):
        # calc coefficient of quintic polynomial
        # See jupyter notebook document for derivation of this equation.
        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[time ** 3, time ** 4, time ** 5],
                      [3 * time ** 2, 4 * time ** 3, 5 * time ** 4],
                      [6 * time, 12 * time ** 2, 20 * time ** 3]])
        b = np.array([xe - self.a0 - self.a1 * time - self.a2 * time ** 2,
                      vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4 + self.a5 * t ** 5

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3 + 5 * self.a5 * t ** 4

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2 + 20 * self.a5 * t ** 3

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t ** 2

        return xt

# parameter
MAX_T = 200.0  # maximum time to the goal [s]
MIN_T = 50.0  # minimum time to the goal[s]

# 5차 방정식의 세 번의 미분을 이용하여 속도가 급격하게 변하지 않는 경로를 생성.
def quintic_polynomials_planner(sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, max_accel, max_jerk, dt):
    vxs = sv * math.cos(syaw)
    vys = sv * math.sin(syaw)
    vxg = gv * math.cos(gyaw)
    vyg = gv * math.sin(gyaw)

    axs = sa * math.cos(syaw)
    ays = sa * math.sin(syaw)
    axg = ga * math.cos(gyaw)
    ayg = ga * math.sin(gyaw)

    time, rx, ry, ryaw, rv, ra, rj = [], [], [], [], [], [], []

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

        if max([abs(i) for i in ra]) <= max_accel and max([abs(i) for i in rj]) <= max_jerk:
            break


    return time, rx, ry, ryaw, rv, ra, rj










################################################################
# 버튼의 실제 작동을 구현하기 위한 코드 부분
################################################################
def draw_button_planning():
    pygame.draw.rect(screen, BUTTON_COLOR_PATH_PLANNING, (BUTTON_X_PATH_PLANNING, BUTTON_Z_PATH_PLANNING, BUTTON_WIDTH, BUTTON_HEIGHT))
    
    text_rect_planning = BUTTON_TEXT_PATH_PLANNING.get_rect()
    text_rect_planning.center = (BUTTON_X_PATH_PLANNING+BUTTON_WIDTH/2,BUTTON_Z_PATH_PLANNING+BUTTON_HEIGHT/2)
    screen.blit(BUTTON_TEXT_PATH_PLANNING, text_rect_planning) #(글자변수, 위치)

def draw_button_tracking():
    pygame.draw.rect(screen, BUTTON_COLOR_PATH_TRACKING, (BUTTON_X_PATH_TRACKING, BUTTON_Z_PATH_TRACKING, BUTTON_WIDTH, BUTTON_HEIGHT))

    text_rect_tracking = BUTTON_TEXT_PATH_TRACKING.get_rect()
    text_rect_tracking.center = (BUTTON_X_PATH_TRACKING+BUTTON_WIDTH/2,BUTTON_Z_PATH_TRACKING+BUTTON_HEIGHT/2)
    screen.blit(BUTTON_TEXT_PATH_TRACKING, text_rect_tracking) #(글자변수, 위치)

def draw_button_reset():
    pygame.draw.rect(screen, BUTTON_COLOR_RESET, (BUTTON_X_RESET, BUTTON_Z_RESET, BUTTON_WIDTH, BUTTON_HEIGHT))
    
    text_rect_planning = BUTTON_TEXT_RESET.get_rect()
    text_rect_planning.center = (BUTTON_X_RESET+BUTTON_WIDTH/2,BUTTON_Z_RESET+BUTTON_HEIGHT/2)
    screen.blit(BUTTON_TEXT_RESET, text_rect_planning) #(글자변수, 위치)

# 버튼이 눌림을 감지하여 해당 기능 수행
def button_control():
    global flag_button_planning_clicked
    global flag_button_tracking_clicked
    global flag_button_reset_clicked
    
    for event in pygame.event.get():
        if event.type == pygame.MOUSEBUTTONDOWN:
            mouse_x, mouse_z = event.pos
            if BUTTON_X_PATH_PLANNING <= mouse_x <= BUTTON_X_PATH_PLANNING + BUTTON_WIDTH \
                and BUTTON_Z_PATH_PLANNING <= mouse_z <= BUTTON_Z_PATH_PLANNING + BUTTON_HEIGHT:
                # 버튼을 클릭했을 때 실행할 동작 추가
                flag_button_planning_clicked=True
                print('Planning Clicked')

            elif BUTTON_X_PATH_TRACKING <= mouse_x <= BUTTON_X_PATH_TRACKING + BUTTON_WIDTH \
                and BUTTON_Z_PATH_TRACKING <= mouse_z <= BUTTON_Z_PATH_TRACKING + BUTTON_HEIGHT:
                # 버튼을 클릭했을 때 실행할 동작 추가
                flag_button_tracking_clicked=True
                print('Tracking Clicked')

            elif BUTTON_X_RESET <= mouse_x <= BUTTON_X_RESET + BUTTON_WIDTH \
                and BUTTON_Z_RESET <= mouse_z <= BUTTON_Z_RESET + BUTTON_HEIGHT:
                # 버튼을 클릭했을 때 실행할 동작 추가
                flag_button_reset_clicked=True
                print('Reset Clicked')

# 주차 구역의 직선 경로를 만들기 위한 1차 함수 생성
def create_linear_function(pos1, pos2):
    # 기울기 계산
    slope = (pos2[1] - pos1[1]) / (pos2[0] - pos1[0])
    
    # y-절편 계산 (하나의 점을 사용하여 계산)
    intercept = pos1[1] - slope * pos1[0]
    
    # 1차 함수를 나타내는 람다 함수 반환
    linear_function = lambda x: slope * x + intercept
    
    return linear_function




# ROS Initialization
rospy.init_node('ar_detect')
motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
motor_msg = xycar_motor()  # 카메라 토픽 메시지

rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
rate = rospy.Rate(10)  # 10Hz로 설정



steering_angle = 0
steering_speed = 7

def drive(angle, speed):
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)

def path_planning(start_pos, entrance_pos):
    # Quintic-Polynomial Planning의 경우 속도정보에 대한 값 설정이 매우 중요하다.
    # 이를 통해 최적의 경로 설정이 가능하기 때문이다.
    # 값이 매우 작으면 경로의 굴곡이 없어 자세 정보를 반영하지 못한다.
    # 반대로 매우 크면 경로의 굴곡이 매우 커져 정상적인 경로 계획으로 볼 수 없다.
    mul_val = 8

    sx = start_pos[0]
    sz = start_pos[1]

    syaw = -math.pi/2 + ar_delta_yaw_rad    # start yaw angle [rad]
    sv = 1.0*mul_val                        # start speed [m/s]
    sa = 0.1                                # start accel [m/ss]

    gx = entrance_pos[0]                    # goal x position [m]
    gz = entrance_pos[1]                    # goal y position [m]
    gyaw = -math.pi/2+edited_ar_yaw_rad     # goal yaw angle [rad]
    gv = 1.0*mul_val                        # goal speed [m/s]
    ga = 0.1                                # goal accel [m/ss]

    max_accel = 1.0*mul_val                 # max accel [m/ss]
    max_jerk = 0.5*mul_val                  # max jerk [m/sss]
    dt = 0.2                                # time tick [s]

    # Path Planning
    time, x, z, yaw_poly, v, a, j = quintic_polynomials_planner(
        sx, sz, syaw, sv, sa, gx, gz, gyaw, gv, ga, max_accel, max_jerk, dt)
    
    return x, z


while not rospy.is_shutdown():
    button_control() # 버튼 클릭을 컨트롤한다.
    draw_background() # 기본 배경 그리기

    
    # 쿼터니언 형식의 자세 정보를 오일러 형식으로 변환
    (_,yaw_radian, _)=euler_from_quaternion((arData["AX"], arData["AY"],
                                            arData["AZ"], arData["AW"]))
    # 라디안 형식의 자세 정보를 각도 형식으로 변환
    yaw_degrees = math.degrees(yaw_radian)
    
    # Pygame 좌표 형식을 맞춰주기 위함.
    # 또한 현재 cm단위로 길이를 출력하고 있기 때문에 
    # m단위 데이터를 cm로 변환 후 픽셀 단위로 변환한다.
    # 1m -> 100cm -> 400pixel
    ar_x_pos = int(arData["DX"]*400) + SCREEN_WIDTH/2
    ar_z_pos = SCREEN_HEIGHT - int(arData["DZ"]*400)

    # AR tag로부터 차량까지의 초기 arctan 각도를 구한다.
    # 2차원상에서의 점과 점으로부터의 각도를 구한 뒤에 Yaw값의 변화량 만큼 더하여 두 점의 자세 정보를 상대적으로 유지한다.
    arctan_angle_cam2ar_rad=math.atan2(int(ar_x_pos-CAR_CAMERA_X),int(CAR_CAMERA_Z-ar_z_pos))

    edited_ar_yaw_rad = yaw_radian + arctan_angle_cam2ar_rad/2

    # 버튼이 눌린 순간의 AR tag의 위치와 자세 정보를 저장한다. 
    # 이를 이용하여 Path의 모양을 유지한다.
    if flag_button_planning_clicked == True:
        ar_delta_x = CAR_CAMERA_X - ar_x_pos # 
        ar_delta_z = CAR_CAMERA_Z - ar_z_pos # always positive
        ar_clicked_yaw_rad = edited_ar_yaw_rad

        flag_show_path=True
        flag_button_planning_clicked=False

    ar_delta_yaw_rad = edited_ar_yaw_rad - ar_clicked_yaw_rad
        
    
    # Planning버튼을 누른 순간의 AR tag의 위치와 자세 정보를 이용하여 가상의 차량 위치를 계산한다. 
    # 즉 AR tag와 가상 차량의 상대적인 위치와 자세를 유지하여 Path의 모양을 유지시킨다.
    moved_car_pos = (
        int(ar_x_pos + math.sqrt(ar_delta_x**2 + ar_delta_z**2) * math.cos(ar_delta_yaw_rad+math.atan2(ar_delta_z, ar_delta_x))),
        int(ar_z_pos + math.sqrt(ar_delta_x**2 + ar_delta_z**2) * math.sin(ar_delta_yaw_rad+math.atan2(ar_delta_z, ar_delta_x))),
    )
    
    
    # 경로를 생성하여 반환해주는 함수
    path_planned_x, path_planned_z = path_planning(moved_car_pos, (parking_slot_entrance_x, parking_slot_entrance_z))

    # AR tag의 토픽이 날아온 첫 순간부터 경로 및 AR tag 그리기 및 바퀴 조향
    if flag_ar_detected == True:
        draw_ar(ar_x_pos , ar_z_pos)  # 차량으로부터의 위치, 자세정보를 계산하여 ar 그리기

        if flag_show_path == True:
            draw_path(path_planned_x,path_planned_z)

    
    draw_wheel_angle_rad = -math.pi/2
    
    # 트래킹 버튼이 눌린 순간부터 Look-Ahead Point를 향해 주행
    if flag_button_tracking_clicked == True:
        error_angle_rad = math.atan2(look_ahead_point_dx, look_ahead_point_dz)
        error_angle_degree = math.degrees(error_angle_rad)
        
        steering_angle = (-error_angle_degree)*3
        steering_angle = max(-100, min(steering_angle, 100)) # -100 ~ 100
        
        if arData["DZ"]*100 < 35:
            drive(0,0) 
        else:
            drive(steering_angle , steering_speed) 

        # 바퀴의 각도를 그려주기 위한 변수
        error_angle_rad = max(-math.pi/9, min(error_angle_rad, math.pi/9)) # -20' ~ +20'
        draw_wheel_angle_rad = error_angle_rad - math.pi/2


        if flag_button_reset_clicked == True:
            drive(0,0)
            flag_button_tracking_clicked = False
            flag_button_reset_clicked = False
            flag_show_path = False
            
    # 조향 각도가 설정된 차량 그리기
    draw_car(draw_wheel_angle_rad)
    


    # 화면 업데이트
    pygame.display.update()
    rate.sleep()

# 게임 종료
pygame.quit()
sys.exit()
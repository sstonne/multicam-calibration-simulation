#!/usr/bin/python
# -*- coding: utf-8 -*-

from i611_MCS import * # 로봇제어 기본기능
from i611_extend import * # 확장기능 사용 (pallet)
from rbsys import * # 관리 프로그램 사용
from i611_common import * # i611Robot 클래스 메소드의 예외처리
from i611_io import * # I/O 신호를 제어
from i611shm import *  # 공유메모리에 액세스
from zeusteach import * # 티칭펜던트 사용
from threading import Event # event 모듈
import threading # thread 모듈
import pdb # 1줄씩 확인 모듈
import time # 시간관련 모듈
import os # 운영체제 모듈
import math # 계산관련 모듈

def shm_din(port): # 입력신호 읽기, port = in_signal number
    return (int(shm_read(0x0100,1))>>port) & 0x01

def th_stop(event):   # thread 함수
    while not event.is_set():
        if din(9) == '1': # din(9) = 1일 때 프로그램 종료
            print("""\
                User Program Stop!""")
            pid = os.getpid()
            os.kill(pid, signal.SIGTERM)    #os signal.SIGKILL
        time.sleep(0.1)

def out(index, num): # Digital Output 제어
    dout(index, str(num)) # index 포트부터 연속적인 포트 제어, num 에 입력받은 숫자로 비트 제어
    time.sleep(0.2)

def current_joint_coordi(): # Joint 좌표계 현재 위치
    joint_list = shm_read( 0x3050, 6 ).split( ',' ) 
    joint_list[0] = round(math.degrees(float(joint_list[0])),3) # Joint 좌표계 J1 위치
    joint_list[1] = round(math.degrees(float(joint_list[1])),3) # Joint 좌표계 J2 위치
    joint_list[2] = round(math.degrees(float(joint_list[2])),3) # Joint 좌표계 J3 위치
    joint_list[3] = round(math.degrees(float(joint_list[3])),3) # Joint 좌표계 J4 위치
    joint_list[4] = round(math.degrees(float(joint_list[4])),3) # Joint 좌표계 J5 위치
    joint_list[5] = round(math.degrees(float(joint_list[5])),3) # Joint 좌표계 J6 위치

    # joint location 출력 
    print("""\
                            Joint 좌표계
                        -----------------------------------------------------------
                            j1      |   {0} deg 
                            j2      |   {1} deg
                            j3      |   {2} deg
                            j4      |   {3} deg
                            j5      |   {4} deg
                            j6      |   {5} deg
                        -----------------------------------------------------------""".format(\
                joint_list[0], joint_list[1], joint_list[2], joint_list[3], joint_list[4], joint_list[5]))

def current_xy_coordi(): # Base 좌표계 현재 위치
    position_list = shm_read( 0x3000, 6).split(',')
    position_list[0] = round(float(position_list[0])*1000,3) # 베이스 좌표계 X 위치
    position_list[1] = round(float(position_list[1])*1000,3) # 베이스 좌표계 Y 위치
    position_list[2] = round(float(position_list[2])*1000,3) # 베이스 좌표계 Z 위치
    position_list[3] = round(math.degrees(float(position_list[3])),3) # 베이스 좌표계 Rz 위치
    position_list[4] = round(math.degrees(float(position_list[4])),3) # 베이스 좌표계 Ry 위치
    position_list[5] = round(math.degrees(float(position_list[5])),3) # 베이스 좌표계 Rx 위치
    posture = shm_read( 0x3040, 1).split(',')
    position_list.append(int(posture[0])) # posture 값, list에 추가
    
    # position location 출력
    print("""\
                            베이스 좌표계
                        -----------------------------------------------------------
                            x       |   {0} mm
                            y       |   {1} mm
                            z       |   {2} mm
                            rz      |   {3} deg
                            ry      |   {4} deg
                            rx      |   {5} deg
                            posture |   {6}
                        -----------------------------------------------------------""".format(\
                position_list[0], position_list[1], position_list[2], position_list[3], position_list[4], position_list[5], position_list[6]))

def input_check(data): # 입력 데이터 타입 검사
    data_trans1 = data.replace("-", "")
    data_trans2 = data_trans1.replace(".", "")
    if data_trans2.isdigit() == False:
                            print("""\
                        입력 데이터 타입이 올바르지 않습니다. 실수형 데이터로 다시 입력해 주세요. 
                        입력 데이터 : {}""".format(data))
    return data_trans2.isdigit()

def dist_deg_fnc(type): # Relmove 거리 또는 각도값 입력
    while True:
        dist_deg_value = raw_input("""\
                    * [Relmove] 움직이고자 하는 거리 또는 각도 입력 (뒤로가기 = back): """)

        if dist_deg_value == 'back':
            break
        elif input_check(dist_deg_value) == False:
            continue
        else:
            print("""\
                            {0} 축으로 {1} 거리 또는 각도 만큼 이동""".format(type, dist_deg_value))
        return dist_deg_value

def main(): # Main Program
    ### 초기 설정 ####################################
    # i611 로봇 생성자
    rb = i611Robot()
    # 베이스 좌표계 정의
    _BASE = Base()
    # 로봇과 연결 시작
    rb.open()
    # I/O 입출력 기능 초기화
    IOinit()

    # 티치 파일 생성
    zt = ZeusTeach()

    ### 티치 파일 내 티치데이터 생성 및 불러오기 ####################################
    ''' 
    각 변수 당 10개, 총 100개 까지 생성 가능
    Joint_Start[0] ~ Joint_Start[9] / Pick_A[0] ~ Pick_A[9] / Place_A[0] ~ Place_a[9] 생성
    '''
    Joint_Start = zt.TJoint(10)
    Pick_A = zt.TPosition(10)
    Place_A = zt.TPosition(10)

    ### 티치데이터 직접 생성 #####################
    '''  Position(x, y, z, rz, ry, rx)'''
    p1 = Position(x=200, y=-280, z=240, rz=90, ry=0, rx=180, posture=7)
    p2 = Position(95, -280, 425, -120, 0, 180, posture=7)
    p3 = Position(300, -280, 240, 159, 86, -156)
    '''  Joint(j1, j2, j3, j4, j5, j6)'''
    j1 = Joint(j1=90, j2=90, j3=-100, j4=180, j5=0, j6=0)
    j2 = Joint(90, 0, -50, 90, 0, 0)

    ### 동작 조건 설정      #####################
    ''' 
    MotionParam 으로 동작 조건 리스트 생성 후 rb.motionparam 으로 SYSTEM에 적용하는 구조, 적용하지 않을 시 default 값으로 적용
    jnt_speed : rb.move, rb.reljntmove 속도에 적용
    lin_speed : rb.line, rb.relline 속도에 적용
    acctime, dacctime : jnt_speed & lin_speed 가 적용되는 모든 이동 명령들에 적용, 가감속 시간이 빠를수록 동작 시간은 감소하나 진동 또는 부하가 심해질 수 있음
    pose_speed : 자세(posture)가 바뀌는 구간의 속도에 적용
    '''
    MP1 = MotionParam(jnt_speed=10, lin_speed=150, pose_speed=20, acctime=0.4, dacctime=0.4, overlap=20)
    MP2 = MotionParam(jnt_speed=30, lin_speed=300, acctime=0.2, dacctime=0.2, overlap=40)
    rb.motionparam(MP1) # 위에서 설정한 motionparameter -> 로봇 적용

    ### 예외 발생 설정     ######################
    '''동작 또는 일시정지 중에 감속 정지 또는 비상 정지 입력 시 예외 발생 유무를 SYSTEM에 적용, 적용하지 않을 시 비활성화'''
    rb.enable_interrupt(0,True) # 동작 중 감속 정지 입력 시의 예외 발생을 활성화
    rb.enable_interrupt(1,True) # 동작 중 비상 정지 입력 시의 예외 발생을 활성화
    rb.enable_interrupt(2,True) # 일시정지 중 감속 정지 입력 시의 예외 발생을 활성화
    rb.enable_interrupt(3,True) # 일시정지 중 비상 정지 입력 시의 예외 발생을 활성화

    ### Thread 설정       ######################
    event=Event()
    th = threading.Thread(target=th_stop, args=(event, )) # def된 함수를 thread 생성
    th.setDaemon(True) # main 함수와 같이 시작하고 끝나도록 daemon 함수로 설정 (병렬동작이 가능하도록 하는 기능)
    th.start() # thread 동작
    
    ### 로봇 동작 정의 ##############################
    # 초기 동작
    out(50, 0) # 50번 포트부터 0 으로 비트 제어, 50번 포트 0 출력
    out(48, 000) # 48번 포트부터 000 으로 비트 제어, 50번 포트 0 & 49번 포트 0 & 48번 포트 0 출력

    # 실제 동작
    try:
        while True: # 사용자의 종료 시퀀스 또는 예외 발생 이외에 무한 반복
            input_data = raw_input("""\
                    * [Main] 실행하고자 하는 Mode 입력 (도움말 = h or help) : """)
            if input_data == 'exit': # 메인 프로그램 종료
                break

            elif input_data == 'h' or input_data == 'help': # 예제출력
                print ("""\
                    -----------------------------------------------------------------------
                        Mode            |   설명 
                    -----------------------------------------------------------------------
                        movement        |   로봇 동작
                        motion_param    |   모션파라미터
                        relmove         |   상대이동
                        pallet          |   Pallet
                        io_pass         |   IO를 활용한 동작 제어
                        mdo             |   bit_field
                        out             |   출력 포트 제어
                        pdb             |   프로그램 디버깅
                        status          |   상태 불러오기
                        location        |   위치값 불러오기
                        get_motion      |   모션파라미터 불러오기
                        tr_coord        |   좌표값 변환 (Joint <-> Position)
                        exit            |   프로그램 종료
                    -----------------------------------------------------------------------""")

            elif input_data == 'movement': # 로봇 동작
                while True:
                    move_type = raw_input("""\
                    * [Movement] 실행하고자 하는 Mode 입력 (도움말 : h or help) : """)
                    if move_type == 'back':
                        break

                    elif move_type == 'h' or move_type == 'help':
                        print("""\
                    -----------------------------------------------------------------------
                        Mode            |   설명 
                    -----------------------------------------------------------------------
                        home            |   Home 위치 이동 (각 Joint 0 위치)
                        move            |   PTP 동작 (jnt_speed 적용)
                        line            |   직선 보간 동작 (lin_speed 적용)
                        back            |   뒤로가기
                    -----------------------------------------------------------------------""")

                    elif move_type == 'home':
                        print("""\
                        각 Joint 값이 0 위치인 Home 위치로 이동""")
                        rb.home()

                    elif move_type == 'move':
                        print("""\
                        PTP 로 Pick_A[0] -> Place_A[0] 동작 수행""")
                        rb.move(Pick_A[0])
                        rb.move(Place_A[0])

                    elif move_type == 'line':
                        print("""\
                        직선으로 Pick_A[0] -> Place_A[0] 동작 수행 (lin_speed 적용)""")
                        rb.line(Pick_A[0])
                        rb.line(Place_A[0])

                    else:
                        print("""\
                        입력 데이터가 올바르지 않습니다. 다시 입력해 주세요. 
                        입력 데이터 : {}""".format(move_type))
                    rb.sleep(0.1)

            elif input_data == 'motion_param': # 모션 파라미터
                while True:
                    mp_default = rb.getmotionparam() # motion_parameter 획득
                    mp = mp_default.mp2list() # parameter -> list 변환

                    param_type = raw_input("""\
                    * [Motion_param] 실행하고자 하는 Mode 입력 (도움말 : h or help) : """)
                    if param_type == 'back':
                        break

                    elif param_type == 'h' or param_type == 'help': #명령예제 출력
                        print("""\
                    -----------------------------------------------------------------------
                        Mode            |   설명                           
                    -----------------------------------------------------------------------
                        lin_speed       |   직선 보간 동작 속도 (default = 5.0 mm/s)
                        jnt_speed       |   PTP 및 최적 직선 보간 동작 속도 (default = 5.0 %)
                        acctime         |   가속 시간 (default = 0.4 s)
                        dacctime        |   감속 시간 (default = 0.4 s)
                        overlap         |   오버랩 거리 (default = 0.0 mm)
                        back            |   뒤로가기
                    -----------------------------------------------------------------------""")

                    elif param_type == 'lin_speed':
                        print("""\
                    -----------------------------------------------------------------------
                        현재 lin_speed = {} mm/s
                    -----------------------------------------------------------------------""".format(mp[0]))

                        value = raw_input("""\
                    * [Lin_speed] 속도값 입력 (mm/s) : """)
                        if value == "back" or input_check(value) == False:
                            continue

                        mcopy = mp_default.copy(lin_speed = float(value))
                        rb.motionparam(mcopy) # 위에서 설정한 motionparameter -> 로봇 적용
                        print("""\
                        {}mm/s 속도로 직선 동작 수행 (Pick_A[0] -> Place_A[0])""".format(value))
                        rb.sleep(1)
                        rb.line(Pick_A[0])
                        rb.line(Place_A[0])

                    elif param_type == 'jnt_speed':
                        print("""\
                    -----------------------------------------------------------------------
                        현재 jnt_speed = {} %
                    -----------------------------------------------------------------------""".format(mp[1]))
                        value = raw_input("""\
                    * [Jnt_speed] 속도값 입력 (%) : """)

                        if value == "back" or input_check(value) == False:
                            continue

                        mcopy = mp_default.copy(jnt_speed = float(value))
                        rb.motionparam(mcopy) # 위에서 설정한 motionparameter -> 로봇 적용
                        print("""\
                        {}% 속도로 PTP 동작 수행 (Pick_A[0] -> Place_A[0])""".format(value))
                        rb.sleep(1)
                        rb.move(Pick_A[0])
                        rb.move(Place_A[0])

                    elif param_type == 'acctime' or param_type == 'dacctime':
                        print("""\
                    -----------------------------------------------------------------------
                        현재 acctime = {}s, dacctime = {}s
                    -----------------------------------------------------------------------""".format(mp[2], mp[3]))
                        value = raw_input("""\
                    * [Acctime and Dacctime] 가감속값 입력 (s) : """)

                        if value == "back" or input_check(value) == False:
                            continue

                        mcopy = mp_default.copy(acctime = float(value), dacctime = float(value))
                        rb.motionparam(mcopy) # 위에서 설정한 motionparameter -> 로봇 적용
                        print("""\
                        {}s 가감속으로 PTP 동작 수행 (Pick_A[0] -> Place_A[0])""".format(value))
                        rb.sleep(1)
                        rb.move(Pick_A[0])
                        rb.move(Place_A[0])

                    elif param_type == 'overlap': # 로봇 예측동작 사용, tact 감축관련 중요 method
                        print("""\
                    -----------------------------------------------------------------------
                        현재 overlap = {}mm
                    -----------------------------------------------------------------------""".format(mp[6]))
                        value = raw_input("""\
                    * [Overlap] 오버랩 거리 입력 (mm) : """)

                        if value == "back" or input_check(value) == False:
                            continue

                        mcopy = mp_default.copy(overlap = float(value))
                        rb.motionparam(mcopy) # 위에서 설정한 motionparameter -> 로봇 적용
                        print("""\
                        비교 전 기준 위치 이동""")
                        rb.sleep(1)
                        rb.move(Place_A[0].offset(dz = 150))
                        print("""\
                        {}mm 오버랩 거리를 적용하여 비교 동작 수행""".format(value))
                        rb.sleep(1)
                        # overlap 미적용
                        # tact 계산시작
                        non_overlap_start_time = time.time()
                        # Pos1 GET
                        rb.move(Pick_A[0].offset(dz = 150))
                        rb.line(Pick_A[0])
                        rb.line(Pick_A[0].offset(dz = 150))
                        # Pos2 PUT
                        rb.move(Place_A[0].offset(dz = 150))
                        rb.line(Place_A[0])
                        rb.line(Place_A[0].offset(dz = 150))
                        # tact 계산종료, 출력
                        non_overlap_tact_time = time.time() - non_overlap_start_time
                        non_overlap_tact_time = round(non_overlap_tact_time,3) # 소숫점 3자리까지만 출력
                        print("""\
                        Overlap 미적용 T/T : {}""".format(non_overlap_tact_time))

                        # overlap 적용
                        # tact 계산시작
                        overlap_start_time = time.time()
                        rb.asyncm(1) # overlap 활성화
                        # Pos1 GET 
                        rb.move(Pick_A[0].offset(dz = 150))
                        rb.line(Pick_A[0])
                        rb.join()
                        rb.line(Pick_A[0].offset(dz = 150))
                        # Pos2 PUT
                        rb.move(Place_A[0].offset(dz = 150))
                        rb.line(Place_A[0])
                        rb.join()
                        rb.asyncm(2) # overlap 비활성화
                        rb.line(Place_A[0].offset(dz = 150))
                        # tact 계산종료, 출력
                        overlap_tact_time = time.time() - overlap_start_time
                        overlap_tact_time = round(overlap_tact_time,3) # 소숫점 3자리까지만 출력
                        print("""\
                        Overlap 적용 T/T : {}""".format(overlap_tact_time))

                    else:
                        print("""\
                        입력 데이터가 올바르지 않습니다. 다시 입력해 주세요. 
                        입력 데이터 : {}""".format(param_type))
                    rb.sleep(0.1)

            elif input_data == 'relmove': # 좌표계 상대이동
                while True:
                    current_joint_coordi()
                    current_xy_coordi()

                    coordi_type = raw_input("""\
                    * [Relmove] 움직이고자 하는 Coordinate 입력 (도움말 = h or help) : """)

                    if coordi_type == 'back':
                        break

                    elif coordi_type == 'h' or coordi_type == 'help':
                        print("""\
                    -----------------------------------------------------------
                        Coordinate      |   설명 
                    -----------------------------------------------------------
                        x               |   x 축 이동
                        y               |   y 축 이동
                        z               |   z 축 이동
                        rz              |   rz 축 이동
                        ry              |   ry 축 이동
                        rx              |   rx 축 이동
                        j1              |   j1 축 이동
                        j2              |   j2 축 이동
                        j3              |   j3 축 이동
                        j4              |   j4 축 이동
                        j5              |   j5 축 이동
                        j6              |   j6 축 이동
                        back            |   뒤로가기
                    -----------------------------------------------------------""")

                    elif coordi_type == 'x':
                        value = dist_deg_fnc(coordi_type)
                        rb.relline(dx=float(value))
                    elif coordi_type == 'y':
                        value = dist_deg_fnc(coordi_type)
                        rb.relline(dy=float(value))
                    elif coordi_type == 'z':
                        value = dist_deg_fnc(coordi_type)
                        rb.relline(dz=float(value))
                    elif coordi_type == 'rz':
                        value = dist_deg_fnc(coordi_type)
                        rb.relline(drz=float(value))
                    elif coordi_type == 'ry':
                        value = dist_deg_fnc(coordi_type)
                        rb.relline(dry=float(value))
                    elif coordi_type == 'rx':
                        value = dist_deg_fnc(coordi_type)
                        rb.relline(drx=float(value))

                    elif coordi_type == 'j1':
                        value = dist_deg_fnc(coordi_type)
                        rb.reljntmove(dj1=float(value))
                    elif coordi_type == 'j2':
                        value = dist_deg_fnc(coordi_type)
                        rb.reljntmove(dj2=float(value))
                    elif coordi_type == 'j3':
                        value = dist_deg_fnc(coordi_type)
                        rb.reljntmove(dj3=float(value))
                    elif coordi_type == 'j4':
                        value = dist_deg_fnc(coordi_type)
                        rb.reljntmove(dj4=float(value))
                    elif coordi_type == 'j5':
                        value = dist_deg_fnc(coordi_type)
                        rb.reljntmove(dj5=float(value))
                    elif coordi_type == 'j6':
                        value = dist_deg_fnc(coordi_type)
                        rb.reljntmove(dj6=float(value))
                    
                    else:
                        print("""\
                        입력 데이터가 올바르지 않습니다. 다시 입력해 주세요. 
                        입력 데이터 : {}""".format(coordi_type))
                    rb.sleep(0.1)

            elif input_data == 'pallet': # pallet 동작, 20개 반송 
                # pallet 1번 생성
                pal1 = Pallet() # pallet 생성 method
                pal1.init_4(Pick_A[0], Pick_A[1], Pick_A[2], Pick_A[3], 5, 4) # 4점 포인트 바운더리 생성_i=5/j=4, total 20cell 구역
                pal1.adjust( 3, 2, 0.4, -0.3 ) # 3,2번 셀 x축 +0.4mm/ y축 -0.3mm 보정

                # pallet 2번 생성
                pal2 = Pallet() # pallet 생성 method
                pal2.init_4(Place_A[0], Place_A[1], Place_A[2], Place_A[3], 5, 4) # 4점 포인트 바운더리 생성_i=5/j=4, total 20cell 구역
                pal2.adjust( 1, 0, 0.4, -0.3 ) # 1,0번 셀 x축 +0.4mm/ y축 -0.3mm 보정
                print("""\
                        pallet 동작 시작!""")
                rb.sleep(1)
                for i in range(5): # i열 5줄
                    for j in range(4): # j열 4줄
                        ## 픽업부 #########################
                        print("""\
                        i = {0}, j = {1}""".format(i,j)) # 작업할 i, j열 위치값 출력
                        pallet1 = pal1.get_pos(i,j) # pal1번 i, j 위치데이터 불러오기
                        rb.move(pallet1.offset(dz=30)) # 목표위치 z.offset +30mm
                        rb.line(pallet1) # 목표위치 도달

                        # clamp close
                        out(48, 1)
                        out(48, 0)

                        rb.line(pallet1.offset(dz=60))

                        ## 플레이스부 ######################
                        pallet2 = pal2.get_pos(i,j) # pal2번 i, j 위치데이터 불러오기
                        rb.move(pallet2.offset(dz=60)) # 목표위치 z.offset +60mm
                        rb.line(pallet2) # 목표위치 도달

                        # clamp open
                        out(50, 1)
                        out(50, 0)

                        rb.line(pallet2.offset(dz=30))
                print("""\
                        pallet 동작 종료!""")

            elif input_data == 'io_pass': # in 신호 확인 후 다음 스텝 진행, 조건문 활용
                print("""\
                        io_pass 동작 시작!""")
                rb.sleep(1)
                while shm_din(7) == 0:
                    ############### 기본 동작 1 cycle  #############################
                    rb.move(Pick_A[0].offset(dz=30))
                    rb.line(Pick_A[0])

                    # clamp close
                    out(48, 1)
                    out(48, 0)

                    print("""\
                        8번 입력이 1이 되면 다음 동작 진행""")
                    while shm_din(8) == 0: # 8번 in_signal on시 다음 step 진행
                        rb.sleep(0.1)
                        pass

                    rb.line(Pick_A[0].offset(dz=50))
                    rb.move(Place_A[0].offset(dz=50))
                    rb.line(Place_A[0])

                    # clamp open
                    out(50, 1)
                    out(50, 0)

                    print("""\
                        8번 입력이 0이 되면 다음 동작 진행""")
                    while shm_din(8) == 1: # 8번 in_signal off시 다음 step 진행
                        rb.sleep(0.1)
                        pass

                    rb.line(Place_A[0].offset(dz=30))
                    rb.sleep(0.001)

            elif input_data == 'mdo': # mdo 기능, 특정구간에서 출력신호 제어
                print("""\
                        mdo 동작 준비위치 이동""")
                rb.sleep(1)
                rb.move(Pick_A[0]) # 출발 위치
                dout(24, '1')

                rb.set_mdo( 1, 24, 0, 1, 200 ) # 출발 위치 기준 200mm 떨어진 구간에서 24번 출력 "0" 로 제어 [관리번호 1번에 등록]
                rb.set_mdo( 8, 24, 1, 2, 200 ) # 도착 위치 기준 200mm 근접한 구간에서 24번 출력 "1" 로 제어 [관리번호 8번에 등록]
                rb.enable_mdo(129) # MDO 관리 번호 1, 8 활성화  1000 0001
                print("""\
                        mdo동작 시작!""")
                rb.sleep(1)
                rb.move(Place_A[1]) # 도착 위치
                rb.disable_mdo(129) # MDO 관리 번호１, ８ 비활성화
                print("""\
                        mdo동작 종료!""")

            elif input_data == 'out': # 출력 포트 제어
                while True:
                    out_data = raw_input("""\
                    * [Out] 제어하고자 하는 시작포트 및 비트 입력 (예시 : 16,000) : """)
                    if out_data == 'back':
                        break
                    else:
                        index, num = out_data.split(",")
                        out(int(index), num)
                    rb.sleep(0.1)

            elif input_data == 'pdb': # 프로그램 디버깅용 메소드
                print("""\
                        한스텝씩 동작 'n', pdb 기능 종료 'c' """)
                pdb.set_trace() # pdb 기능 / 입력값_ c = continue (pdb 종료), n = next (한라인씩 스텝동작)
                out(48, 1)
                out(48, 0)
                out(50, 1)
                out(50, 0)

            elif input_data == 'status': # 로봇 시스템 상태확인
                system_list = rb.get_system_port()
                print("""\
                    -----------------------------------------------------------
                        Running        |   {0}
                        Svon           |   {1}
                        Emo            |   {2}
                        Hw_error       |   {3}
                        Sw_error       |   {4}
                        Abs_lost       |   {5}
                        In_pause       |   {6}
                        Error          |   {7}
                    -----------------------------------------------------------""".format(\
                        system_list[0], system_list[1], system_list[2], system_list[3], system_list[4], system_list[5], system_list[6], system_list[7]))
                
            elif input_data == 'location': # 현재 매니퓰레이터 좌표 출력
                current_joint_coordi()
                current_xy_coordi()
                    
            elif input_data == 'get_motion': # motion_parameter 불러오기
                mp = rb.getmotionparam() # motion_parameter 획득
                mp = mp.mp2list() # parameter -> list 변환
                print("""\
                    -----------------------------------------------------------
                        lin_speed       |   {0} mm/s
                        jnt_speed       |   {1} %
                        acctime         |   {2} s
                        dacctime        |   {3} s
                        posture         |   {4}
                        passm           |   {5}
                        overlap         |   {6} mm
                        zone            |   {7} pulse
                        pose_speed      |   {8} %
                        ik_solver_option|   {9}
                    -----------------------------------------------------------""".format(\
                        round(mp[0],1), round(mp[1],1), round(mp[2],1), round(mp[3],1), int(mp[4]), int(mp[5]), round(mp[6],1), int(mp[7]), round(mp[8],1), mp[9]))
        
            elif input_data == 'tr_coord': # coordinate 변환 -- 현재위치 좌표 변환으로 변경 할 것
                print("""\
                        현재위치를 좌표계 변환""")
                tr_pos = rb.getpos()

                jnt = rb.Position2Joint(tr_pos) # position -> joint 변환
                jnt_list = jnt.jnt2list()
                print("""\
                        Joint 좌표계
                    -----------------------------------------------------------
                        j1      |   {0} deg 
                        j2      |   {1} deg
                        j3      |   {2} deg
                        j4      |   {3} deg
                        j5      |   {4} deg
                        j6      |   {5} deg
                    -----------------------------------------------------------""".format(\
                        round(jnt_list[0],3), round(jnt_list[1],3), round(jnt_list[2],3), round(jnt_list[3],3), round(jnt_list[4],3), round(jnt_list[5],3)))
                
                pos = rb.Joint2Position(jnt)         # joint    -> position 변환
                pos_list = pos.pos2list()
                print("""\
                        베이스 좌표계
                    -----------------------------------------------------------
                        x       |   {0} mm
                        y       |   {1} mm
                        z       |   {2} mm
                        rz      |   {3} deg
                        ry      |   {4} deg
                        rx      |   {5} deg
                        posture |   {6}
                    -----------------------------------------------------------""".format(\
                        round(pos_list[0],3), round(pos_list[1],3), round(pos_list[2],3), round(pos_list[3],3), round(pos_list[4],3), round(pos_list[5],3), pos_list[7]))

            else:
                print("""\
                        입력 데이터가 올바르지 않습니다. 다시 입력해 주세요. 
                        입력 데이터 : {}""".format(input_data))

            rb.sleep(0.1)

    except KeyboardInterrupt: # "ctrl" + "c" 버튼 입력
        print("""
                    KeyboardInterrupt""")
    except Robot_emo: # EMO push
        print("""
                    Robot_emo 누름""")
    except Exception as e: # 다른 예외
        print("""
                    Error name is : {}""".format(e))
    finally: # try 구문 종료 시 반드시 실행
        print("""\
                    Finish""")
        dout(48, '000')

    ### 종료 ######################################
    # 로봇과의 연결을 종료
    event.set()
    th.join()
    rb.close()

if __name__ == '__main__':
    main()

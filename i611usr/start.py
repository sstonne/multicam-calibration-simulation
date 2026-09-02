#!/usr/bin/python
# -*- coding: utf-8 -*-

## 1. 초기 설정① #######################################
# 라이브러리 가져오기
## 1．초기 설정 ①　모듈 가져오기 ######################
from i611_MCS import *
from teachdata import *
from i611_extend import *
from rbsys import *
from i611_common import *
from i611_io import *
from i611shm import * 

def main():
    
    ## 2. 초기 설정② ####################################
    # i611 로봇 생성자
    rb = i611Robot()
    # 좌표계의 정의
    _BASE = Base()
    # 로봇과 연결 시작 초기화
    rb.open()
    # I/O 입출력 기능의 초기화 
    IOinit( rb )
    # 교시 데이터 파일 읽기
    data = Teachdata( "teach_data" )
    ## 1. 교시 포인트 설정 ######################
    p1 = data.get_position( "pos1", 0 )
    p2 = data.get_position( "pos1", 1 )

    ## 2. 동작 조건 설정 ######################## 
    m = MotionParam( jnt_speed=20, lin_speed=200, pose_speed=100, overlap = 30 )
    #MotionParam 형으로 동작 조건 설정
    rb.motionparam( m )
    
    ## 3. 로봇 동작을 정의 ##############################
    startJ = Joint(194.84,  15.23,  84.83,-179.89, -80.15,-72.2)
    rb.move(startJ)	
    ## 4. 종료 ######################################
    # 로봇과의 연결을 종료
    rb.close()

if __name__ == '__main__':
    main()
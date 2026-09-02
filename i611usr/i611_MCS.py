# -*- coding: utf-8 -*-
u"""Robot control module （ロボットを制御するモジュール）

**Class Hierarchy Chart （クラス階層図）**

.. image:: i611_MCS.png
"""

import threading, thread
import time
import os
import sys
import socket
import atexit,signal
import math
import traceback

import rblib
from errlog import liblog, liberr, liberr_config, get_syserr_msg, liberr_sub
import i611_common
from i611_common import Robot_emo,Robot_stop,Robot_forcesensor,Robot_error,Robot_fatalerror,Robot_poweroff,Robot_exception
import rbsys
from i611shm import shm_read

#[Internal] --------------------------
# Multiturnを使うかどうか （mtが使えるのは、0.3.6.0以降)
_use_mt = False #: Private (非公開)

#[Internal] --------------------------
def version_i611_MCS():
  u"""Private (非公開)"""
  return [0, 3, 7, 0]


"""======================================================================
[ダミークラス]
  ワールド座標系  Positionクラス、Coordinateクラスで利用するダミークラス
matrix(self):	ダミー 単位行列を返す
eulerangle(self):	[0 0 0] ベクトルを返す
======================================================================"""
class Base(object):
  u"""Definition of the WORLD coordinate system. A dummy class used for the Position class or the Coordinate class. （ワールド座標系を規定する）"""

  def __init__(self):
    u"""Instantiates a dummy class used for the Position class or the Coordinate class defined in the WORLD
coordinate system by calling the constructor. （コンストラクタ。ワールド座標系の Position クラス、Coordinate クラスで利用するダミークラスのインスタンスを作成する）

    Args:
      None

    Return:
      Base: Reference to instance. （Baseクラスオブジェクトへの参照）

    **Example**::

      _BASE=Base()

    """
    self.__I = i611_common._matEye(4)

  #[Internal] --------------------------
  def matrix(self):
    u"""Private (非公開)"""
    return self.__I

  #[Internal] --------------------------
  def eulerangle(self):
    u"""Private (非公開)"""
    return [0.0, 0.0, 0.0]

_BASE = Base()

"""======================================================================
[内部クラス]
  _ParentContainer : Positionクラス，Coordinateクラスの基底クラス
  matrix()  :  位置・姿勢・親オブジェクトから変換行列
======================================================================"""
#[Internal] --------------------------
class _ParentContainer(object):
  u"""Private (非公開)"""

  pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, _BASE]
  #[Internal] --------------------------
  def matrix(self):
    u"""Private (非公開)"""
    m = i611_common._matEuler(self.pos[3], self.pos[4], self.pos[5])
    m = i611_common._mdotm(i611_common._matShift(self.pos[0], self.pos[1], self.pos[2]), m)
    m = i611_common._mdotm(self.pos[6].matrix(), m)
    return m

"""======================================================================
[内部クラス]
  _WatchStatus : (非常停止およびDigitalINからのコマンド)の監視・動作コマンド実行管理

enableEMO(self, sw):	非常停止状態監視 ON/OFF
run(self):	監視スレッド本体
stop(self):	監視スレッド停止
======================================================================"""
#[Internal] --------------------------
PRINT_PAUSE_STATE=False
ONCE_CREATED = False   # 同一プロセスでインスタンスが作れるのは1回だけ
ONCE_OPENED  = False   # 同一プロセスでopenできるのは1回だけ
class _WatchStatus(threading.Thread):
  u"""Private (非公開)"""

  #############################
  def __init__(self, host, port, rbmcs):
    u"""Private (非公開)"""
    super(_WatchStatus, self).__init__()
    self.setDaemon(True)

    self.EID_NO_ERROR      = 0
    self.EID_SOCKETERR     = 1
    self.EID_HW_ERROR      = 2
    self.EID_SW_ERROR      = 3
    self.EID_ABSLOST       = 4
    self.EID_INVALIDMODE   = 5
    self.EID_INVALIDTHREAD = 6
    self.EID_OTHERS        = 99
    self._fatal_error      = 0
    self._power_off        = 0
    self._ev_threadend     = threading.Event()
    self._ev_threadend.set()                # set before thread start

    self.__pid = os.getpid()
    self.__i611rb = rbmcs                   ## i611Robot class (already opened)
    self.__rblib = rblib.Robot(host, port) ## Rblib
    self.__rblib.open()

    # チェックは、RobSysを作る前に実施
    if ONCE_CREATED:
      code = 19
      print "cause fatal exit(%d:%s)" % (code, get_syserr_msg(10,code))
      self.__write_app_status(2,code)
      os._exit(1)
#      self.__cause_fatal_exit(19)
#      code = 19
#      print "cause fatal exit(%d:%s)" % (code, self._get_syserr_msg(10,code))
#      self.__write_app_status(2,code)

    self.__robsys = rbsys.RobSys(host)      ## RObSys to communicate with i611sys
    try:
      self.__robsys.open()
    except Exception as exc:
      self._fatal_error = self.EID_SW_ERROR
      raise exc

    # request for app value
    self.REQ_NONE       = 0x00
    self.REQ_STOP       = 0x01
    self.REQ_PAUSE      = 0x02
    self.REQ_CONTINUE   = 0x04
    self.REQ_KILL       = 0x08

    # timeout after EMO
    self.alive_timeout_sec = 5

    self._enbErr          = True         # Raise Exception in error
    self._disableDin      = False
    self._teachMode       = False
    self._monitorMode     = False
    self._PauseDuringMove = False

    self._ev_stop        = threading.Event()
    self._ev_fs_detect   = threading.Event()
    self._ev_fatal_error = threading.Event()
    self._ev_emo         = threading.Event()
    self._ev_pause       = threading.Event()
    self._ev_continue    = threading.Event()

    # behavior parameter
    self._f_enable_only_user_hook = False
    self._f_servo_off_during_pause = False
    self._f_restore_pos_before_continue = False
    self._f_no_pause_during_moving = False

    # interrupt parameter
    self._f_enb_interrupt_stop = False
    self._f_enb_interrupt_emo  = False
    self._f_enb_interrupt_stop_in_pause = False
    self._f_enb_interrupt_emo_in_pause  = False

    # information in shared memory
    self.reg_val = {}
    self._cur_system_status = 0
    self._cur_error_code    = 0

    # Pause status value
    self.PS_NONE      = 0  # 000:not pause
    self.PS_RCV_PAUSE = 1  # 001:recv pause command
    self.PS_IN_MOVE   = 3  # 011:paused in native
    self.PS_IN_HOOK   = 5  # 101:paused in hook
    self.PS_IN_BOTH   = 7  # 111:paused in both native and hook
    self.PS_RCV_CONT  = 6  # 110:recv continue command
    self.PS_OUT_MOVE  = 4  # 100:resume native
    self.PS_OUT_HOOK  = 2  # 010:resume hook
    self._cur_pause_status  = self.PS_NONE
    self.pause_lock = threading.Lock()

  #############################
  def __del__(self):
    u"""Private (非公開)"""
    self.thread_end()
    self.__robsys.close()
    self.__rblib.close()

  #############################
  def abort(self):
    u"""Private (非公開)"""
    return self.__rblib.abortm()

  #############################
  def ioctrl(self, wordno, dataL, maskL, dataH, maskH):
    u"""Private (非公開)"""
    return self.__rblib.ioctrl(wordno, dataL, maskL, dataH, maskH)

  #############################
  def thread_end(self):
    u"""Private (非公開)"""
    if not self._ev_threadend.is_set():
      self._ev_threadend.set()
      self.join(0.5)

  #############################
  def register_program(self):
    u"""Private (非公開)"""
    pid = os.getpid()
    return self.__robsys.register_program(pid)

  #############################
  ## for system status
  def ss_update_info(self):
    u"""Private (非公開)"""
    reg = 0x0308
    self.reg_val[reg] = int(shm_read(reg,1))

  def ss_get_system_status(self):
    u"""Private (非公開)"""
    sts = (self.reg_val[0x0308] >> 4) & 0x0F
    eid = (self.reg_val[0x0308] >> 8) & 0x0FF
    return sts, eid

  def get_door_status(self):
    u"""Private (非公開)"""
    reg = 0x0115
    reg_val = int(shm_read(reg,1))
    door = (reg_val >> 4) & 0x03
    return door

  #[Internal] --------------------------
  def __write_app_status(self,status,code,pause_status=0):
    u"""Private (非公開)"""
    st_app_status = 0
    if status>-1:
      st_app_status = ((status&0x0F) << 24)
    if code>-1:
      st_app_status |= (code&0x0FF) << 16
    st_app_status |= (pause_status&0x07) << 29
    self.__rblib.ioctrl(130, st_app_status, 0x0000ffff, 0, 0xffffffff)

  #############################
  def run(self):
    u"""Private (非公開)"""
    # $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    # App Background thread Main Loop

    try:
      self._ev_threadend.clear()
      while not self._ev_threadend.is_set():

        sts = self.__robsys.req_mcmd()
        # req_mcmd state
        # 0  status - running
        # 1  status - svon
        # 2  status - emo
        # 3  status - hw_error
        # 4  status - sw_error
        # 5  status - abs_lost
        # 6  status - in_pause
        # 7  status - error
        # 8  request for app (bit mask)
        if sts[2] == 1 and self._enbErr and not self._ev_emo.is_set():
          self._ev_emo.set()    # Emp Exception
          #pylint: disable=maybe-no-member
          signal.alarm(self.alive_timeout_sec)
  #        print "EMO set alarm"

        if sts[8] & self.REQ_STOP and not self._ev_stop.is_set():
          self._ev_stop.set()    # Stop Exception
          self.abort()
          # clear other events
          self._ev_pause.clear()
          self._ev_continue.clear()

        elif sts[8] & self.REQ_PAUSE and not self._ev_pause.is_set():
          with self.pause_lock:
            self._cur_pause_status = self.PS_RCV_PAUSE
            self.__write_app_status(0,-1,self._cur_pause_status)
          if PRINT_PAUSE_STATE:
            print "PS_RCV_PAUSE"

          # pause thread
          self._ev_pause.set()

          # pause motion
          if not self._f_no_pause_during_moving:
            if PRINT_PAUSE_STATE:
              print "CALL:rblib.suspendm()"
            ret = self.__rblib.suspendm(1)
            if PRINT_PAUSE_STATE:
              print "suspendm()=",ret
            # Check shm
            ReqSuspend, Suspended = shm_read(0x317E,2).split(',')
            ReqSuspend = int(ReqSuspend)
            Suspended = int(Suspended)
            if PRINT_PAUSE_STATE:
              print " SUS: ReqSuspend=%d, Suspended=%d" % (ReqSuspend,Suspended)
            #if ReqSuspend == Suspended:
              #print "ERROR! Suspend Fail(Suspended = Suspended)"
            if True or ret[0] and ret[1]!=3: # (Always suspended in native) Pause in native
              # Enter pause state
              with self.pause_lock:
                self._cur_pause_status |= 0x02 #PS_IN_MOVE
                self.__write_app_status(4,-1,self._cur_pause_status)
              if PRINT_PAUSE_STATE:
                print "PS_IN_MOTION"
              liblog("%d = i611Robot.suspend()" % ret[2])
            else:
              if PRINT_PAUSE_STATE:
                print "not stop in native"
              liblog("(None) = i611Robot.suspend()")

          # clear other events
          self._ev_stop.clear()
          self._ev_continue.clear()

        elif sts[8] & self.REQ_CONTINUE and not self._ev_continue.is_set():
          with self.pause_lock:
            self._cur_pause_status = self.PS_RCV_CONT
            self.__write_app_status(0,-1,self._cur_pause_status)
          if PRINT_PAUSE_STATE:
            print "PS_RCV_CONT"

          # clear event before resume
          self._ev_stop.clear()
          self._ev_pause.clear()

          # continue motion ( always call resumem )
          if PRINT_PAUSE_STATE:
            print "CALL:rblib.resumem()"
          ret = self.__rblib.resumem()
          if PRINT_PAUSE_STATE:
            print "resumem()=",ret
          # check shm
          ReqSuspend, Suspended = shm_read(0x317E,2).split(',')
          ReqSuspend = int(ReqSuspend)
          Suspended = int(Suspended)
          if PRINT_PAUSE_STATE:
            print " RES: ReqSuspend=%d, Suspended=%d" % (ReqSuspend,Suspended)
          if ReqSuspend !=0 or Suspended!=0:
            print "ERROR! Resume Fail(Suspended = Suspended)"
          with self.pause_lock:
            self._cur_pause_status &= ~0x02 #PS_OUT_MOVE
            self.__write_app_status(0,-1,self._cur_pause_status)
          if PRINT_PAUSE_STATE:
            print "PS_NONE"

          # continue thread
          self._ev_continue.set()

        # Check SystemStatus
        self.ss_update_info()
        self._cur_system_status, self._cur_error_code = self.ss_get_system_status()

        # check jog mode
        if not self._teachMode and not os.path.exists("/tmp/auto_ready") and self._enbErr:
          if self.get_door_status() == 3:
            if not self._ev_emo.is_set():
              self._ev_emo.set()    # Emp Exception
              #pylint: disable=maybe-no-member
              signal.alarm(self.alive_timeout_sec)
          else:
            self._fatal_error = self.EID_INVALIDMODE

        # Critical error
        if sts[3] == 1 and self._enbErr:
          if self._cur_error_code != 0: # エラーIDが取得できるまで無視
            self._fatal_error = self.EID_HW_ERROR
            if  self._cur_error_code == 98: # PowerOff
              self._power_off = 1
        elif sts[4] == 1 and self._enbErr:
          self._fatal_error = self.EID_SW_ERROR
        elif sts[5] == 1 and self._enbErr:
          self._fatal_error = self.EID_ABSLOST
        if self._fatal_error != 0:
          break


        ## Loop wait (50ms)
        time.sleep(0.02)

    except Exception as e:
      sys.stderr.write("_WatchStatus.run caused exception\n")
      traceback.print_exc()
      raise e
    finally:
      if self._fatal_error != 0 and not self._ev_fatal_error.is_set(): # thread exit
#        print "_WatchStatus Exit(%d)" % self._fatal_error
        self._ev_fatal_error.set()
        self.abort()

    # App Background thread Main Loop End
    # $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

"""======================================================================
[クラス]
  Coordinate
  直交座標系で定義されたCoordinateクラスを生成する

■ メソッド(関数)
__param(self, *arg1, **arg2):	可変個数引数、キーワード引数を解釈する
replace(self, *arg1, **arg2):	パラメータ置き換え
shift(self, *arg1, **arg2):	パラメータのシフト
copy(self):	オブジェクトコピー
clear(self):	パラメータの初期化
inv(self): 逆変換を行うCoordinateクラスのオブジェクトコピーを返す

■　メンバ変数
  pos  :  座標系パラメータ

■ 戻り値備考
  replace  :  なし
  shift  :  なし

■ 内部メソッド(関数)
  __param : コンストラクタ,replace関数の引数処理
======================================================================"""
class Coordinate(_ParentContainer):
  u"""Handling the WORLD coordinate system （ワールド座標系オブジェクトを扱う）"""

  __defpos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, _BASE]

  def __init__(self, *arg1, **arg2):
    u"""Instantiates a coordinate class defined in the WORLD coordinate system. （コンストラクタ。ワールド座標系で定義されたCoordinate クラスのインスタンスを作成する）

    Args:
      x(float): Location (in the WORLD coordinate system) （X位置（ワールド座標））
      y(float): Location (in the WORLD coordinate system) （Y位置（ワールド座標））
      z(float): Location (in the WORLD coordinate system) （Z位置（ワールド座標））
      rz(float): Orientation (Euler angles in the Z-Y-X system) （Rz姿勢（Z-Y-X 系オイラー角））
      ry(float): Orientation (Euler angles in the Z-Y-X system) （Ry姿勢（Z-Y-X 系オイラー角））
      rx(float): Orientation (Euler angles in the Z-Y-X system) （Rx姿勢（Z-Y-X 系オイラー角））
      parent(Coordinate or Base): (Set transformation matrix to convert location and orientation
       data, using Coordinate and Position, to the data in the WORLD coordinate system)
       （ワールド座標系を使用する設定）

    Return:
      Coordinate: Reference to instance. （Coordinateクラスオブジェクトへの参照）

    **Example**::

      # Ex1: Omit parameters (use default value)
      # 例1：引数省略( 初期値を設定する)
      CO1=Coordinate()

      # Ex2: Specify all parameters
      # 例2：キーワード( 全て)
      CO2=Coordinate( x=1, y=2, z=3, rz=1, ry=2, rx=3, parent=_BASE )

      # Ex3: Specify 6 parameters
      # 例3：キーワード( ６個)
      CO3=Coordinate( x=1, y=2, z=3, rz=1, ry=2, rx=3 )

      # Ex4: Specifiy only a prameter
      # 例4：キーワード( １個)、指定しなかった値は初期値になります
      CO4=Coordinate( x=10 )

    """
    self.pos = Coordinate.__defpos
    self.__param(*arg1, **arg2)

  def __param(self, *arg1, **arg2):
    u"""Private (非公開)"""
    #pram [x, y, z, rz, ry, rx, parent]
    p = i611_common._args(self.pos,
      ['x', 'y', 'z', 'rz', 'ry', 'rx', 'parent'],
      [float, float, float, float, float, float, None],
      *arg1,
      **arg2)

    if p[0] == False:
      return liberr(4,p[1],"Coordinate.__param")
    else:
      self.pos = p[1:]
      if not isinstance(self.pos[6],_ParentContainer) and not isinstance(self.pos[6],Base):
        return liberr(4,4,"Coordinate.__param")
      Coordinate.__defpos = self.pos


  #[Public] ############################
  def replace(self, *arg1, **arg2):
    u"""Replaces a teaching point and updates self. （ワールド座標系オブジェクトを置換する（自身を更新する））

    Args:
      x(float): Location (in the WORLD coordinate system) （X位置（ワールド座標））
      y(float): Location (in the WORLD coordinate system) （Y位置（ワールド座標））
      z(float): Location (in the WORLD coordinate system) （Z位置（ワールド座標））
      rz(float): Orientation (Euler angles in the Z-Y-X system) （Rz姿勢（Z-Y-X 系オイラー角））
      ry(float): Orientation (Euler angles in the Z-Y-X system) （Ry姿勢（Z-Y-X 系オイラー角））
      rx(float): Orientation (Euler angles in the Z-Y-X system) （Rx姿勢（Z-Y-X 系オイラー角））
      parent(Coordinate or Base): (Set transformation matrix to convert location and orientation
       data, using Coordinate and Position, to the data in the WORLD coordinate system) 
       （ワールド座標系を使用する設定）

    Return:
      Coordinate: Reference to instance. （Coordinateクラスオブジェクトへの参照）

    **Example**::

      # Ex: Specify 2 parameters. Default value are used for the others.
      # 例：リスト(2 個)　指定しなかった値は初期値になります
      CO2=Coordinate()
      CO2.replace( 7, 8 )

    """
    self.__param(*arg1, **arg2)
    return self


  #[Public] ############################
  def shift(self, *arg1, **arg2):
    u"""Offsets coordinate parameters and updates self. （ワールド座標系オブジェクトをシフトする（自身を更新する））

    Args:
      dx(float): X-axis offset (in the WORLD coordinate system) （X位置シフト量（ワールド座標））
      dy(float): Y-axis offset (in the WORLD coordinate system) （Y位置シフト量（ワールド座標））
      dz(float): Z-axis offset (in the WORLD coordinate system) （Z位置シフト量（ワールド座標））
      drz(float): rz-axis offset  (Rz姿勢シフト量（Z-Y-X 系オイラー角））
      dry(float): ry-axis offset  (Ry姿勢シフト量（Z-Y-X 系オイラー角））
      drx(float): rx-axis offset （Rx姿勢シフト量（Z-Y-X 系オイラー角））

    Return:
      Coordinate: Reference to instance （Coordinateクラスオブジェクトへの参照）

    Note:
      * This api changed the instance itself. （このAPIは、オブジェクト自身を更新します）

    **Example**::

      CO1 = Coordinate()
      CO1.replace( 1, 2, 3, 4, 5, 6 )
      CO1.shift( 80, 70 )


    """
    p = i611_common._args([0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              ['dx', 'dy', 'dz', 'drz', 'dry', 'drx'],
              [float, float, float, float, float, float],
              *arg1,
              **arg2)
    if p[0] == False:
      return liberr(4,p[1],"Coordinate.__param")
    else:
      self.pos[0:6] = [self.pos[i] + p[i+1] for i in range(6)]
      return self


  #[Public] ############################
  def copy(self):
    u"""Copies a coordinate object.　（ワールド座標系オブジェクトをコピーする）

    Args:
      None

    Return:
      Coordinate: A new object of the copy (Coordinate object) （コピーしたCoordinateクラスオブジェクトへの参照）

    **Example**::

      # copy CO1 into CO1C
      #CO1 を新しいCoordinate オブジェクトCO1C にコピーする
      CO1=Coordinate( x=1, y=2, z=3, rz=4, ry=5, rx=6, parent=_BASE )
      CO1C=CO1.copy()

    """
    p = Coordinate(self.pos[:])
    return p

  #[Public] ############################
  def inv(self):
    u"""Initiate instance for inverse transformation （逆変換を行うCoordinate クラスのオブジェクトを生成する）

    Args:
      None

    Return:
      Coordinate: Reference to instance  （新しく生成した逆変換を行うCoordinate オブジェクト）

    **Example**::

      # Create position in WORLD coordinate system.
      # ワールド座標系のティーチングポイントを作成する。
      CO1 = Coordinate()
      CO1.replace( 1, 2, 3, 4, 5, 6 )

      # create inverse transformation object
      # 逆変換オブジェクトを生成する
      CO2 = CO1.inv()

    """
    m = i611_common._minv(self.matrix())
    mw = [m[0][3], m[1][3], m[2][3]] + i611_common._eulMatrix(m)
    p = Coordinate(mw[0:6],_BASE)
    return p

  #[Public] ############################
  def g2b(self, X,Y,Z,RZ,RY,RX):
    u"""Convert WORLD coordinates to BASE coordinates. （ワールド座標系からベース座標系へ変換する）

    Args:
       * X(float): X-axis location (in the WORLD coordinate system) （X位置（ワールド座標））
       * Y(float): Y-axis location (in the WORLD coordinate system) （Y位置（ワールド座標））
       * Z(float): Z-axis location (in the WORLD coordinate system) （Z位置（ワールド座標））
       * RZ(float): rz-axis angle (in the WORLD coordinate system) （Rz姿勢（Z-Y-X 系オイラー角））
       * RY(float): ry-axis angle (in the WORLD coordinate system)  （Ry姿勢（Z-Y-X 系オイラー角））
       * RX(float): rx-axis angle (in the WORLD coordinate system)  （Rx姿勢（Z-Y-X 系オイラー角））

    Return:
      list: [x, y, z, rz, ry, rx]

        * x(float): X-axis location (in the BASE coordinate system) （X位置（ベース座標））
        * y(float): Y-axis location (in the BASE coordinate system) （Y位置（ベース座標））
        * z(float): Z-axis location (in the BASE coordinate system) （Z位置（ベース座標））
        * rz(float): rz-axis angle (in the BASE coordinate system) （Rz姿勢（Z-Y-X 系オイラー角））
        * ry(float): ry-axis angle (in the BASE coordinate system) （Ry姿勢（Z-Y-X 系オイラー角））
        * rx(float): rx-axis angle (in the BASE coordinate system) （Rx姿勢（Z-Y-X 系オイラー角））

    **Example**::

      CO1=Coordinate()
      CO1.g2b( 1, 2, 3, 4, 5, 6 )

    """
    mp = i611_common._mdotm(i611_common._matShift(X,Y,Z), i611_common._matEuler(RZ,RY,RX))
    mw = i611_common._minv(self.matrix())
    m = i611_common._mdotm(mw, mp)
    return [m[0][3], m[1][3], m[2][3]] + i611_common._eulMatrix(m)

  #[Public] ############################
  def b2g(self, x,y,z,rz,ry,rx):
    u"""Convert BASE coordinates to WORLD coordinates （ベース座標系からワールド座標系へ変換する）

    Args:
        * x(float): X-axis location (in the BASE coordinate system) （X位置（ベース座標））
        * y(float): Y-axis location (in the BASE coordinate system) （Y位置（ベース座標））
        * z(float): Z-axis location (in the BASE coordinate system) （Z位置（ベース座標））
        * rz(float): rz-axis angle (in the BASE coordinate system) （Rz姿勢（Z-Y-X 系オイラー角））
        * ry(float): ry-axis angle (in the BASE coordinate system) （Ry姿勢（Z-Y-X 系オイラー角））
        * rx(float): rx-axis angle (in the BASE coordinate system) （Rx姿勢（Z-Y-X 系オイラー角））

    Return:
      list: [x, y, z, rz, ry, rx]

       * X(float): X-axis location (in the WORLD coordinate system) （X位置（ワールド座標））
       * Y(float): Y-axis location (in the WORLD coordinate system) （Y位置（ワールド座標））
       * Z(float): Z-axis location (in the WORLD coordinate system) （Z位置（ワールド座標））
       * RZ(float): rz-axis angle (in the WORLD coordinate system) （Rz姿勢（Z-Y-X 系オイラー角））
       * RY(float): ry-axis angle (in the WORLD coordinate system)  （Ry姿勢（Z-Y-X 系オイラー角））
       * RX(float): rx-axis angle (in the WORLD coordinate system)  （Rx姿勢（Z-Y-X 系オイラー角））

    **Example**::

      CO1=Coordinate()
      CO1.b2g( 1, 2, 3, 4, 5, 6 )

    """
    mp = i611_common._mdotm(i611_common._matShift(x,y,z), i611_common._matEuler(rz,ry,rx))
    mw = self.matrix()
    m = i611_common._mdotm(mw, mp)
    return [m[0][3], m[1][3], m[2][3]] + i611_common._eulMatrix(m)

  #[Public] ############################
  def clear(self):
    u"""Initializes the coordinate system parameters. （ワールド座標系のオブジェクトを初期化する）

    Args:
      None

    Return:
      None

    **Example**::

      CO1=Coordinate( 1, 2, 3, 4, 5, 6,_BASE )
      CO1.clear()

    """
    self.pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, _BASE]
    Coordinate.__defpos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, _BASE]

"""======================================================================
[クラス]
  Positionクラス
  直交座標系で定義された教示点を定義する

■ メソッド(関数)
__param(self, *arg1, **arg2):	可変個数引数、キーワード引数を解釈する
replace(self, *arg1, **arg2):	パラメータ置き換え
offset(self, **val):	パラメータオフセット
shift(self, **val):	パラメータのシフト
copy(self):	オブジェクトのコピー
clear(self):	パラメータの初期化
pos2list(self):	内部で保持しているパラメータをリスト形式で出す
pos2dict(self):	内部で保持しているパラメータを辞書形式で出す
position(self):	親座標系に変換した教示点をリスト形式で返す

■　メンバ変数
  pos  :  教示点パラメータ [x, y, z, rz, ry, rx, parent, posture, multiturn]

■ 戻り値備考
  replace  :  Positionオブジェクト
  offset  :  Positionオブジェクト
  shift  :  Positionオブジェクト

■ 内部メソッド(関数)
  __param : コンストラクタ,replace関数の引数処理
======================================================================"""
class Position(_ParentContainer):
  u"""Provides the properties and methods for teaching points defined in the WORLD coordinate system. （ワールド座標系のPosition 座標値を扱う）"""

  __defpos = [0., 0., 0., 0., 0., 0., _BASE, -1, 0xFF000000L]

  def __init__(self, *arg1, **arg2):
    u"""Creates an instance object to define a teaching point defined in the WORLD coordinate system and
calls the constructor. （コンストラクタ。ワールド座標系のティーチングポイントを定義するインスタンスを作成する）

    Args:
      x(float): Location (in the WORLD coordinate system) （X位置（ワールド座標））
      y(float): Location (in the WORLD coordinate system) （Y位置（ワールド座標））
      z(float): Location (in the WORLD coordinate system) （Z位置（ワールド座標））
      rz(float): Orientation (Euler angles in the Z-Y-X system) （Rz姿勢（Z-Y-X 系オイラー角））
      ry(float): Orientation (Euler angles in the Z-Y-X system) （Ry姿勢（Z-Y-X 系オイラー角））
      rx(float): Orientation (Euler angles in the Z-Y-X system) （Rx姿勢（Z-Y-X 系オイラー角））
      parent(Coordinate or Base): (Set transformation matrix to convert location and orientation data, using Coordinate and Position, to the data in the WORLD coordinate system)  (ワールド座標系を使用する設定）
      posture(int): Posture parameter:0 - 7 （postureパラメータ: 0 - 7）
      multiturn(long): Crossover counter （クロスオ―バーカウンタ情報）

    Return:
      Position:  Reference to instance. （Positionクラスオブジェクトへの参照）

    **Example**::

      # Ex1: Omit parameter (Use default).
      # 例1：引数省略( 初期値を設定する)
      P1=Position()

      # Ex2: Specify parameters by keywords.
      # 例2：キーワード
      P2=Position( x=1, y=2, z=3, rz=1, ry=2, rx=3, parent=_BASE, posture=1 )

      # Ex3: Specify a parameter (Use default for the other parameters)
      # 例3：キーワード( １個) を指定しなかった値は初期値になります
      P3=Position( rx=103 )

    """
    self.pos = Position.__defpos
    self.__param(*arg1, **arg2)

  def __param(self, *arg1, **arg2):
    u"""Private (非公開)"""
    #pram [x, y, z, rz, ry, rx, parent, posture, multiturn]
    p = i611_common._args(self.pos,
              ['x', 'y', 'z', 'rz', 'ry', 'rx', 'parent', 'posture', 'multiturn'],
              [float, float, float, float, float, float, None, int, long],
              *arg1, **arg2)

    if p[0] == False:
      return liberr(4,p[1],"Position.__param")
    else:
      self.pos = p[1:]
      if isinstance(self.pos[6],int):
        self.pos[8] = self.pos[7]
        self.pos[7] = self.pos[6]
        self.pos[6] = Position.__defpos[6]
      Position.__defpos = self.pos

  #[Public] ############################
  def replace(self, *arg1, **arg2):
    u"""Position 座標値を置換する

    Args:
      x(float): Location (in the WORLD coordinate system) （X位置（ワールド座標））
      y(float): Location (in the WORLD coordinate system) （Y位置（ワールド座標））
      z(float): Location (in the WORLD coordinate system) （Z位置（ワールド座標））
      rz(float): Orientation (Euler angles in the Z-Y-X system) （Rz姿勢（Z-Y-X 系オイラー角））
      ry(float): Orientation (Euler angles in the Z-Y-X system) （Ry姿勢（Z-Y-X 系オイラー角））
      rx(float): Orientation (Euler angles in the Z-Y-X system) （Rx姿勢（Z-Y-X 系オイラー角））
      parent(Coordinate or Base): (Set transformation matrix to convert location and orientation
       data, using Coordinate and Position, to the data in the WORLD coordinate system) 
       （ワールド座標系を使用する設定）
      posture(int): Posture parameter:0 - 7 （postureパラメータ: 0 - 7）
      multiturn(long): Crossover counter （クロスオ―バーカウンタ情報）

    Return:
      Position:  Reference to instance. （Positionクラスオブジェクトへの参照）

    Note:
      * This API update itself.
      * このAPIは、オブジェクト自身を更新します

    **Example**::

      P1=Position()
      P2=Position()
      P3=Position()

      P1.replace( 1, 2, 3, 4, 5, 6 )

      P2.replace( 7, 8 )

      P3.replace( x=1, rx=6 )

    """
    self.__param(*arg1, **arg2)
    return self

  #[Public] ############################
  def offset(self, *arg1, **arg2):
    u"""Position 座標値をシフトしたオブジェクトを生成する（自身を更新しない）

    Args:
      dx(float): X-axis offset (in the WORLD coordinate system) （X位置シフト量（ワールド座標））
      dy(float): Y-axis offset (in the WORLD coordinate system) （Y位置シフト量（ワールド座標））
      dz(float): Z-axis offset (in the WORLD coordinate system) （Z位置シフト量（ワールド座標））
      drz(float): rz-axis offset  (Rz姿勢シフト量（Z-Y-X 系オイラー角））
      dry(float): ry-axis offset  (Ry姿勢シフト量（Z-Y-X 系オイラー角））
      drx(float): rx-axis offset （Rx姿勢シフト量（Z-Y-X 系オイラー角））

    Return:
      Position: Reference to new instance （新しいPositionクラスオブジェクトへの参照）

    Note:
      * This API doesn't update itself. （このAPIは、オブジェクト自身を更新しません。）
      * Except returning new instance, the function is the same as shift(). （新しいオブジェクトを返すこと以外は、shift()と同じです。）

    **Example**::

      P1=Position( x=1, y=2, z=3, rz=4, ry=5, rx=6 )

      P1ofs=P1.offset( dx=100, drz=-10 )

    """
    p = self.copy()
    return p.shift(*arg1, **arg2)

  #[Public] ############################
  def shift(self, *arg1, **arg2):
    u"""Position 座標値をシフトする（自身を更新する）

    Args:
      dx(float): X-axis offset (in the WORLD coordinate system) （X位置シフト量（ワールド座標））
      dy(float): Y-axis offset (in the WORLD coordinate system) （Y位置シフト量（ワールド座標））
      dz(float): Z-axis offset (in the WORLD coordinate system) （Z位置シフト量（ワールド座標））
      drz(float): rz-axis offset  (Rz姿勢シフト量（Z-Y-X 系オイラー角））
      dry(float): ry-axis offset  (Ry姿勢シフト量（Z-Y-X 系オイラー角））
      drx(float): rx-axis offset （Rx姿勢シフト量（Z-Y-X 系オイラー角））

    Return:
      Position: Reference to instance （Positionクラスオブジェクトへの参照）

    Note:
      * This API updates itself. （このAPIは、オブジェクト自身を更新します）

    **Example**::

      P1=Position( 1, 2, 3, 4, 5, 6 )

      P1.shift(dx=100, drz=-10)

    """
    p = i611_common._args([0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              ['dx', 'dy', 'dz', 'drz', 'dry', 'drx'],
              [float, float, float, float, float, float],
              *arg1, **arg2)
    if p[0] == False:
      return liberr(4,p[1],"Position.shift")
    else:
      self.pos[0:6] = [self.pos[i] + p[i+1] for i in range(6)]
      return self

  #[Public] ############################
  def copy(self):
    u"""Copies a teaching point. （Position 座標値をコピーする）

    Args:
      None

    Return:
      Position: A new object of the copied teaching point (Position object) （コピーしたPositionクラスオブジェクトへの参照）

    **Example**::

      P1=Position( x=1, y=2, z=3, rz=4, ry=5, rx=6, 0xFF000000 )

      P1C=P1.copy()

    """
    return Position(self.pos[:])

  #[Public] ############################
  def clear(self):
    u"""Initializes teaching point data. （Position 座標値を初期化する）

    Args:
      None

    Return:
      None

    Note:
      Default value （初期値）: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, _BASE,-1, 0xFF000000]

    **Example**::

      P1=Position( 1, 2, 3, 4, 5, 6 )

      P1.clear()

    """
    self.pos = [0., 0., 0., 0., 0., 0., _BASE, -1, 0xFF000000L]
    Position.__defpos = [0., 0., 0., 0., 0., 0., _BASE, -1, 0xFF000000L]

  #[Public] ############################
  def pos2list(self):
    u"""Outputs a teaching point in the list format. （Position 座標値をリスト形式で取得する）

    Args:
      None

    Return:
      list: [x, y, z, rz, ry, rx, parent, posture, (multiturn)]

    Note:
      * If i611Robot.use_mt() is set to True, crossover counter is added in return value.
      * （i611Robot.use_mt(True) を設定している場合は、戻り値にmultiturn の項目が追加されます。）
      * If i611Robot.use_mt() is set to False, crossover counter is NOT added in return value.
      * （i611Robot.use_mt(False)（初期値）のときは、追加されません。）

    **Example**::

      P1=Position( 1, 2, 3, 4, 5, 6 )

      P1.pos2list()

    """
    if _use_mt:
      return self.pos[:]
    else: # _not_mt
      return self.pos[:8]

  #[Public] ############################
  def pos2dict(self):
    u"""Outputs a teaching point in the Dictionary format. （Position 座標値を辞書形式で取得する）

    Args:
      None

    Return:
      dict: {'x', 'y', 'z', 'rz', 'ry', 'rx', 'parent', 'posture', ('multiturn')}

    Note:
      * If i611Robot.use_mt() is set to True, crossover counter is added in return value.
      * （i611Robot.use_mt(True) を設定している場合は、戻り値にmultiturn の項目が追加されます。）
      * If i611Robot.use_mt() is set to False, crossover counter is NOT added in return value.
      * （i611Robot.use_mt(False)（初期値）のときは、追加されません。）

    **Example**::

      P1=Position( 1, 2, 3, 4, 5, 6 )

      P1.pos2dict()

    """
    if _use_mt:
      k = ['x', 'y', 'z', 'rz', 'ry', 'rx', 'parent', 'posture', 'multiturn']
    else: # _not_mt
      k = ['x', 'y', 'z', 'rz', 'ry', 'rx', 'parent', 'posture']
    return dict(zip(k, self.pos))

  #[Public] ############################
  def position(self,force_use_mt=False):
    u"""Outputs a teaching point converted into the parent coordinate system in the List format. （Position 座標値を親座標系に変換し、リスト形式で取得する）

    Args:
      force_use_mt(bool): If True, return list contains crossover counter in spite of use_mt() setting.  （Trueのときは、use_mtの値に関係なくクロスオーバーカウンタ値も取得する。）

    Return:
      list: [x, y, z, rz, ry, rx, parent, posture, (multiturn)]

    **Example**::

      P1=Position( 1, 2, 3, 4, 5, 6 )

      P1.position()

    """
    if isinstance(self.pos[6], Base):
      if _use_mt or force_use_mt:
        return self.pos
      else:
        return self.pos[:8]
    mp = i611_common._mdotm(i611_common._matShift(self.pos[0],
      self.pos[1],self.pos[2]), i611_common._matEuler(self.pos[3],self.pos[4],self.pos[5]))
    mw = self.pos[6].matrix()
    m = i611_common._mdotm(mw, mp)
    if _use_mt or force_use_mt:
      return [m[0][3], m[1][3], m[2][3]] + i611_common._eulMatrix(m) + [self.pos[6], self.pos[7], self.pos[8]]
    else: # _not_mt
      return [m[0][3], m[1][3], m[2][3]] + i611_common._eulMatrix(m) + [self.pos[6], self.pos[7]]

  #[Public] ############################
  def has_mt(self):
    u"""Confirm if this position has a crossover counter. （クロスオ―バーカウンタ情報を確認する）

    Args:
      None

    Return:
      bool: Whether crossover counter exists or not.（クロスオーバーカウンタ情報の有無）

        * True: Exists （あり）
        * False: None

    """
    if (self.pos[8] & 0xFF000000L) == 0xFF000000L:
      return False
    else:
      return True

"""======================================================================
[クラス]
  Jointクラス
  ロボットの各軸座標を保存する

■ メソッド(関数)
__param(self, *arg1, **arg2):	可変個数引数、キーワード引数を解釈する
replace(self, *arg1, **arg2):	パラメータ置き換え
offset(self, **val):	パラメータオフセット
shift(self, **val):	パラメータのシフト
copy(self):	オブジェクトのコピー
clear(self):	パラメータの初期化
jnt2list(self):	内部で保持しているパラメータをリスト形式で出す
jnt2dict(self):	内部で保持しているパラメータを辞書形式で出す

■ 戻り値備考
  replace  :  Jointオブジェクト
  offset  :  Jointオブジェクト
  shift  :  Jointオブジェクト

■　メンバ変数
  jnt  :  パラメータ [j1, j2, j3, j4, j5, j6]

■ 内部メソッド(関数)
  __param : コンストラクタ,replace関数の引数処理
  __shift
======================================================================"""
class Joint(object):
  u"""Defines the class to control each coordinate axis of the robot （ジョイント座標系のJoint 座標値の角度データを扱う）"""

  __defJnt = [0., 0., 0., 0., 0., 0.]

  def __init__(self, *arg1, **arg2):
    u"""Creates an instance to save each coordinate axis of the robot and calls the constructor. （コンストラクタ。ジョイント座標系のティーチングポイントを定義するインスタンスを作成する）

    Args:
      j1(float): J1 axis value[Deg] （J1関節角度（Deg））
      j2(float): J2 axis value[Deg] （J2関節角度（Deg））
      j3(float): J3 axis value[Deg] （J3関節角度（Deg））
      j4(float): J4 axis value[Deg] （J4関節角度（Deg））
      j5(float): J5 axis value[Deg] （J5関節角度（Deg））
      j6(float): J6 axis value[Deg] （J6関節角度（Deg））

    Return:
      Joint: Reference to instance  （Jointクラスオブジェクトへの参照）

    **Example**::

      J1=Joint()

      J2=Joint( 1, 1, 1, 1, 1, 1 )

      J3=Joint( j1=1, j2=2, j3=3, j4=4, j5=5, j6=6 )

      J4=Joint( j6 = 6 )

    """
    self.jnt = Joint.__defJnt
    self.__param(*arg1, **arg2)

  def __param(self, *arg1, **arg2):
    u"""Private (非公開)"""
    #pram = [j1, j2, j3, j4, j5, j6]
    p = i611_common._args(self.jnt,
              ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'],
              [float, float, float, float, float, float],
              *arg1, **arg2)
    if p[0] == False:
      return liberr(4,p[1], "Joint.__param")
    else:
      self.jnt = p[1:]
      Joint.__defJnt = self.jnt

  #[Public] ############################
  def replace(self, *arg1, **arg2):
    u"""Replaces each axis coordinate parameter and updates self. （Joint 座標値を置換する）

    Args:
      j1(float): J1 axis value[Deg] （J1関節角度（Deg））
      j2(float): J2 axis value[Deg] （J2関節角度（Deg））
      j3(float): J3 axis value[Deg] （J3関節角度（Deg））
      j4(float): J4 axis value[Deg] （J4関節角度（Deg））
      j5(float): J5 axis value[Deg] （J5関節角度（Deg））
      j6(float): J6 axis value[Deg] （J6関節角度（Deg））

    Return:
      Joint: Reference to instance  （Jointクラスオブジェクトへの参照）

    Note:
      * This api changed the instance itself. （このAPIは、オブジェクト自身を更新します）

    **Example**::

      J1=Joint()
      J2=Joint()
      J3=Joint()

      J1.replace( 1, 2, 3, 4, 5, 6 )

      J2.replace( 7, 8 )

      J3.replace( j1=1, j6=6 )

    """
    self.__param(*arg1, **arg2)
    return self

  #[Public] ############################
  def offset(self, *arg1, **arg2):
    u"""Offsets each axis coordinate parameter, generates a new teaching point object, and keeps self. （Joint 座標値をシフトしたオブジェクトを生成する（自身を更新しない））

    Args:
      dj1(float): J1 axis offset value （J1関節角度シフト量（Deg））
      dj2(float): J2 axis offset value （J2関節角度シフト量（Deg））
      dj3(float): J3 axis offset value （J3関節角度シフト量（Deg））
      dj4(float): J4 axis offset value （J4関節角度シフト量（Deg））
      dj5(float): J5 axis offset value （J5関節角度シフト量（Deg））
      dj6(float): J6 axis offset value （J6関節角度シフト量（Deg））

    Return:
      Joint: Reference to new instance  （Jointクラスオブジェクトへの参照）

    Note:
      * This API doesn't update itself. （このAPIは、オブジェクト自身を更新しません。）
      * Except returning new instance, the function is the same as shift(). （新しいオブジェクトを返すこと以外は、shift()と同じです。）

    **Example**::

      J1=Joint( 1, 2, 3, 4, 5, 6 )

      J1ofs=J1.offset( dj1=80, dj6=-30 )

      J2ofs=J1.offset( 80, -30 )

    """
    p = self.copy()
    return p.shift(*arg1, **arg2)

  #[Public] ############################
  def shift(self, *arg1, **arg2):
    u"""Joint 座標値をシフトする（自身を更新する）

    Args:
      dj1(float): J1 axis offset value （J1関節角度シフト量（Deg））
      dj2(float): J2 axis offset value （J2関節角度シフト量（Deg））
      dj3(float): J3 axis offset value （J3関節角度シフト量（Deg））
      dj4(float): J4 axis offset value （J4関節角度シフト量（Deg））
      dj5(float): J5 axis offset value （J5関節角度シフト量（Deg））
      dj6(float): J6 axis offset value （J6関節角度シフト量（Deg））

    Return:
      Joint: Reference to instance  （Jointクラスオブジェクトへの参照）

    Note:
      * This API updates itself. （このAPIは、オブジェクト自身を更新します）

    **Example**::

      J1.replace( 1, 2, 3, 4, 5, 6 )

      J1.shift( dj1=80 )

    """
    p = i611_common._args([0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              ['dj1', 'dj2', 'dj3', 'dj4', 'dj5', 'dj6'],
              [float, float, float, float, float, float],
              *arg1, **arg2)
    if p[0] == False:
      return liberr(4,p[1], "Joint.shift")
    else:
      self.jnt[0:6] = [self.jnt[i] + p[i+1] for i in range(6)]
      return self

  #[Public] ############################
  def copy(self):
    u"""Copies each axis coordinate. （Joint 座標値をコピーする）

    Args:
      None

    Return:
      Joint: A new object of the copied teaching point (Joint object)　（コピーしたJointクラスオブジェクトへの参照）

    **Example**::

      J1=Joint( j1=1, j2=2, j3=3, j4=4, j5=5, j6=6 )

      J1C=J1.copy()

    """
    return Joint(self.jnt[:])

  #[Public] ############################
  def clear(self):
    u"""Initializes coordinate parameter of each axis. （Joint 座標値を初期化する）

    Args:
      None

    Return:
      None

    Note:
      Default value （初期値）: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    **Example**::

      J1=Joint( 1, 2, 3, 4, 5, 6 )

      J1.clear()

    """
    self.jnt = [0., 0., 0., 0., 0., 0.]
    Joint.__defJnt = [0., 0., 0., 0., 0., 0.]

  #[Public] ############################
  def jnt2list(self):
    u"""Outputs each axis coordinate parameter in the List format.　（Joint 座標値をリスト形式で取得する）

    Args:
      None

    Return:
      list: [j1, j2, j3, j4, j5, j6]

    **Example**::

      J1=Joint( 1, 2, 3, 4, 5, 6 )

      J1.jnt2list()

    """
    return self.jnt[:]

  #[Public] ############################
  def jnt2dict(self):
    u"""Outputs each axis coordinate parameter in the Dictionary format. （Joint 座標値を辞書形式で取得する）

    Args:
      None

    Return:
      dict: {'j1', 'j2', 'j3', 'j4', 'j5', 'j6'}

    **Example**::

      J1=Joint( 1, 2, 3, 4, 5, 6 )

      J1.jnt2dict()

    """
    k = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
    return dict(zip(k, self.jnt[:]))

"""======================================================================
[クラス]
  MotionParamクラス
  ロボットの運動パラメータ

■ メソッド(関数)
__param(self, *arg1, **arg2):	可変個数引数、キーワード引数を解釈する
mp2list(self):	内部で保持しているパラメータをリスト形式で返す
__list2mp(self, mp):	リスト形式で与えられたパラメータを内部の変数に格納する
mp2dict(self):	内部で保持しているパラメータを辞書形式で出す
copy(self,*arg1, **arg2):	オブジェクトをコピーし、さらにパラメータを置き換える
clear(self):	パラメータの初期化
confdefault(self, *arg1, **arg2):	パラメータの初期値を設定する
motionparam(self, *arg1, **arg2):	パラメータを設定する

■ メンバ変数
  lin_speed  :  速度(CP動作 mm/s）
  jnt_speed  :  速度(PTP動作,Joint動作 %)
  pose_speed  :  速度(姿勢補間動作 % 100%=45deg/s)
  acctime  :  加速時間 [s]
  dacctime  :  減速時間 [s]
  passm  :  パス動作パラメータ 1:ON / 2:OFF
  overlap  :  オーバーラップ動作パラメータ[mm]
  posture  :  ポーズ規定値
  zone  :  位置決め整定範囲 [pulse]
  ik_solver_option  :  PotionからJoint変換時の、関節ごとの変換方式指定

■ 戻り値備考
  mp2list : [lin_speed, jnt_speed, acctime, dacctime, posture, passm, overlap, zone, pose_speed, ik_solver_option]

■ 内部メソッド(関数)
  __param : コンストラクタ,motionparam関数の引数処理
  __list2mp  :  動作パラメータをリスト形式で設定する
======================================================================"""
class MotionParam(object):
  u"""Defines robot motion parameters （ロボットの動作パラメータを扱う）"""

  __default = [5.0, 5.0, 0.4, 0.4, 2, 2, 0.0, 100, 20.0, 0x11111111L]

  def __init__(self, *arg1, **arg2):
    u"""Instantiates the parameters of robot motion and calls the constructor. （コンストラクタ。ロボットの動作パラメータクラスのインスタンスを作成する）

    Args:
      lin_speed(float): speed (Liner motion based on linear interpolation)[mm/s] Default:5.0  （速度(Line動作（直線補間動作))[mm/s]（初期値: 5.0））

        * A type of motion based on linear interpolation that follows a straight line path where X-Y-Z axes are synchronously controlled.
        * （X-Y-Z 軸を同期制御しながら目的地までの軌跡が直線になるように一定速で移動する動作）

      jnt_speed(float): speed (PTP motion, Joint motion)[%] Default:5.0 （速度(PTP動作、Joint動作、最適直線補間動作)[%]（初期値: 5.0））

        * Each joint moves at a constant speed and angle to the target position. A type of motion that follows a smooth curved path
        *  （すべての関節が目標座標に向かって、一定速度、角度で動作する。滑らかな曲線を描きながら移動する動作）
        * Optline motion is also controlled by jnt_speed parameter. 100% means maximum speed.
        *  （最適直線補間動作もjnt_speed で速度を設定する。変速するため最高速度に対する％で設定する。）

      acctime(float): acceleration time[s] Default:0.4 （加速時間[s]（初期値: 0.4））

        * lin_speed、 acceleration time to reach specified speed. （設定した速度に到達する時間を設定します。）

      dacctime(float): deceleration time[s] Default:0.4 （減速時間[s]（初期値: 0.4)）

        * lin_speed、 deceleration time to stop from specified speed. （設定した速度から目標の座標に減速停止するまでの時間。）

      posture(int): posture parameter. Default:2　（姿勢（初期値: 2)）

        * Range （設定範囲）: 0 - 7
        * Posture is defined by the combination of arm position and joint value. （マニピュレータの姿勢は、アームの位置と関節角度で定義されます。）

      passm(int): pass motion parameter. Default:2 （パス動作（初期値: 2））

        * Range （設定値）：1=ON 、2=OFF
        * When the passm parameter is ON, the wait time between motions is skipped and the total motion duration will be shortened in the section specified by the asyncm( ) method.
        *  （passm 動作パラメータをON すると、動作間の待機時間をスキップし、全体の動作時間を短縮できます。asyncm(1) オーバーラップ（先読み）を有効にすると、passm 動作パラメータの設定に関係なく常にON と同じ動作になります。）

      overlap(float): overlap motion parameter. Default:0.0 （オーバーラップ距離[mm]（初期値：0.0））

        * The robot starts moving based on the next motion instruction (as such overlapped motion) when it approaches the target. With this function, the robot can keep moving without stopping at a via point set up to avoid obstacles.
        * (目標点(B) に近づいた時点で次の動作が重ね合わされた動作を開始します。障害物を避ける等の動作をさせるために設ける経由地点（B) で、動作を停止させることなくロボットを滑らかに動かすことができます。）

      zone(int):positioning settling range. Default:100 （位置決め完了範囲[pulse]（初期値: 100））

        * During movement of the end of robot arm towards a target point, each of J1 - J6 controls the movement to its value range specified by zone (positioning settling range). This parameter zone helps to improve the accuracy of motion.
        * (ロボットアーム先端が目標点に近づき、位置決め完了判定をするエンコーダパルス範囲を設定します。）

      pose_speed(float): speed (posture interpolated motion)　Default:20.0 （速度[%]（姿勢補間動作）(初期値: 20.0））

        * 100% = 45deg/s
        * マニピュレータ先端が向きを変えながら動作する際の、先端のオイラー角の動作速度の上限を設定します。

      ik_solver_option(long): Each joint rotation value. Default:0x11111111 （回転方向[flag]（初期値: 0x11111111））

        * Specify the joint rotation parameter by corresponding 4bit of bitfield which is refered by motion to a position or conversion from Position to Joint
        * 0: Not refer to the crossover counter
        * 1: Refer to the crossover counter
        * 2: Joint rotates plus side to the target
        * 3: Joint rotates minus side to the target

        * Position 型で指定された座標への動作や、Position型からJoint 型へ座標変換をおこなう際の、各軸の回転方向を4bit毎に指定します。
        * 0: 従来互換動作<multiturn なし>
        * 1: 回転動作<multiturn あり>
        * 2: 指定位置に対して+ 方向に近回りする
        * 3: 指定位置に対してー方向に近回りする

    Return:
      MotionParam: Reference to instance  （MotionParam クラスオブジェクトへの参照）

    **Example**::

      # Ex1: Omit parameter (Use default)
      # 例1：引数省略( 初期値を設定する)
      m=MotionParam()

      # Ex2: Specify some parameters
      # 例2：引数で指定した動作パラメータを設定する
      m=MotionParam( lin_speed=70, jnt_speed=10, overlap=30 )

    """
    self.__list2mp(MotionParam.__default)
    self.__param(*arg1, **arg2)

  def __param(self, *arg1, **arg2):
    u"""Private (非公開)"""
    #pram [lin_speed, jnt_speed, acctime, dacctime, posture, passm, overlap, zone, pose_speed, ik_solver_option]
    p = i611_common._args(self.mp2list(),
              ['lin_speed', 'jnt_speed', 'acctime', 'dacctime', 'posture',
              'passm', 'overlap', 'zone', 'pose_speed', 'ik_solver_option'],
              [float, float, float, float, int, int, float, int, float, long],
              *arg1, **arg2)
    if p[0] == False:
      return liberr(4,p[1],"MotionParam.__param")
    else:
      self.__list2mp(p[1:])

  #[Public] ############################
  def mp2list(self):
    u"""Returns motion parameters in the list format （動作パラメータをリスト形式で取得する）

    Args:
      None

    Return:
      list: [lin_speed, jnt_speed, acctime, dacctime, posture, passm, overlap, zone, pose_speed, ik_solver_option]

    See Also:
      See the explanation of constructor as for each parameter.
      （各パラメータの意味は、コンストラクタの説明を参照して下さい。）

    """
    return [self.lin_speed, self.jnt_speed, self.acctime,
      self.dacctime, self.posture, self.passm, self.overlap, self.zone, self.pose_speed, self.ik_solver_option]

  def __list2mp(self, mp):
    u"""Private (非公開)"""
    res = i611_common._chkparam(mp[0], p_type = [int,float], min = 0.0)
    if res[0] == False:
      return liberr(4, res[1],"MotionParam.__list2mp")
    else:
      self.lin_speed = mp[0]

    res = i611_common._chkparam(mp[1], p_type = [int,float], min = 0.0, max = 100.0)
    if res[0] == False:
      return liberr(4, res[1],"MotionParam.__list2mp")
    else:
      self.jnt_speed = mp[1]

    res = i611_common._chkparam(mp[2], p_type = [int,float], min = 0.0)
    if res[0] == False:
      return liberr(4, res[1],"MotionParam.__list2mp")
    else:
      self.acctime = mp[2]

    res = i611_common._chkparam(mp[3], p_type = [int,float], min = 0.0)
    if res[0] == False:
      return liberr(4, res[1],"MotionParam.__list2mp")
    else:
      self.dacctime = mp[3]

    res = i611_common._chkparam(mp[4], p_type = int, min = -1, max = 7)
    if res[0] == False:
      return liberr(4, res[1],"MotionParam.__list2mp")
    else:
      self.posture = mp[4]

    res = i611_common._chkparam(mp[5], p_type = int, min = 1, max = 2)
    if res[0] == False:
      return liberr(4, res[1],"MotionParam.__list2mp")
    else:
      self.passm = mp[5]

    res = i611_common._chkparam(mp[6], p_type = [int,float], min = 0.0)
    if res[0] == False:
      return liberr(4, res[1],"MotionParam.__list2mp")
    else:
      self.overlap = mp[6]

    res = i611_common._chkparam(mp[7], p_type = int, min = 1)
    if res[0] == False:
      return liberr(4, res[1],"MotionParam.__list2mp")
    else:
      self.zone = mp[7]

    res = i611_common._chkparam(mp[8], p_type = [int,float], min = 1.0)
    if res[0] == False:
      return liberr(4, res[1],"MotionParam.__list2mp")
    else:
      self.pose_speed = mp[8]

    res = i611_common._chkparam(mp[9], p_type = long, min = 0)
    if res[0] == False:
      return liberr(4, res[1],"MotionParam.__list2mp")
    else:
      self.ik_solver_option = mp[9]


  #[Public] ############################
  def mp2dict(self):
    u"""Returns motion parameters in the Dictionary format. （動作パラメータを辞書形式で取得する）

    Args:
      None

    Return:
      dict: {'lin_speed', 'jnt_speed', 'acctime', 'dacctime', 'posture', 'passm', 'overlap', 'zone', 'pose_speed', 'ik_solver_option'}

    See Also:
      See the explanation of constructor as for each parameter.
      （各パラメータの意味は、コンストラクタの説明を参照して下さい。）

    """
    k = ['lin_speed', 'jnt_speed', 'acctime', 'dacctime', 'posture',
      'passm', 'overlap', 'zone', 'pose_speed', 'ik_solver_option']
    return dict(zip(k, self.mp2list()))

  #[Public] ############################
  def copy(self,*arg1, **arg2):
    u"""Returns a copy of the motion parameters with some changes made.　（動作パラメータをコピーする）

    Args:
      * If parameter is omitted, current parameters are copied. （引数を指定すると現在の設定されている動作パラメータを変更してコピーします。）
      * If some parameters are specified, those parameters are overwritten and copied. （引数を省略すると現在の設定されている動作パラメータの値がコピーされます。）

    Return:
      MotionParam: Reference to instance.  （MotionParam クラスオブジェクトへの参照）

    **Example**::

      m=MotionParam( jnt_speed=10, lin_speed=70, overlap=30 )

      # Ex1: Omit parameter (just copy)
      # 例1：引数省略( 現在の設定されている動作パラメータ)
      mcopy = m.copy()

      # Ex2: Specify parameter (copy and overwrite parameter)
      # 例2：引数で指定した動作パラメータを変更してコピーする
      mcopy = m.copy( jnt_speed=15 )

    """
    p = MotionParam(self.mp2list()[:])
    return p.motionparam(*arg1, **arg2)

  #[Public] ############################
  def clear(self):
    u"""Initializes the motion parameters to their default values. （動作パラメータを初期化する）

    Args:
      None

    Return:
      None

    **Example**::

      m=MotionParam( lin_speed=70, jnt_speed=10, overlap=30 )

      m.clear()

    """
    self.__list2mp(MotionParam.__default)

  #[Public] ############################
  def confdefault(self, *arg1, **arg2):
    u"""Sets default values to the motion parameters. （動作パラメータの初期値を変更する）

    Args:
      lin_speed(float): speed (Liner motion based on linear interpolation)[mm/s] Default:5.0  （速度(Line動作（直線補間動作))[mm/s]（初期値: 5.0））
      jnt_speed(float): speed (PTP motion, Joint motion)[%] Default:5.0 （速度(PTP動作、Joint動作、最適直線補間動作)[%]（初期値: 5.0））
      acctime(float): acceleration time[s] Default:0.4 （加速時間[s]（初期値: 0.4））
      dacctime(float): deceleration time[s] Default:0.4 （減速時間[s]（初期値: 0.4)）
      posture(int): posture parameter. Default:2　（姿勢（初期値: 2)）
      passm(int): pass motion parameter. Default:2 （パス動作（初期値: 2））
      overlap(float): overlap motion parameter. Default:0.0 （オーバーラップ距離[mm]（初期値：0.0））
      zone(int):positioning settling range. Default:100 （位置決め完了範囲[pulse]（初期値: 100））
      pose_speed(float): speed (posture interpolated motion)　Default:20.0 （速度[%]（姿勢補間動作）(初期値: 20.0））
      ik_solver_option(long): Each joint rotation value. Default:0x11111111 （回転方向[flag]（初期値: 0x11111111））

    Return:
      None

    See Also:
      See the explanation of constructor as for each parameter.
      （各パラメータの意味は、コンストラクタの説明を参照して下さい。）

    **Example**::

      m.clear()
      m.confdefault( lin_speed=70, overlap=30 )
      m.clear()

    """
    p = i611_common._args(MotionParam.__default,
              ['lin_speed', 'jnt_speed', 'acctime', 'dacctime', 'posture',
              'passm', 'overlap', 'zone', 'pose_speed', 'ik_solver_option'],
              [float, float, float, float, int, int, float, int, float, long],
              *arg1, **arg2)
    if p[0] == False:
      return liberr(4,p[1], "MotionParam.confdefault")
    else:
      MotionParam.__default = p[1:]

  #[Public] ############################
  def motionparam(self, *arg1, **arg2):
    u"""Sets or updates the motion parameters. （動作パラメータを設定する）

    Args:
      lin_speed(float): speed (Liner motion based on linear interpolation)[mm/s] Default:5.0  （速度(Line動作（直線補間動作))[mm/s]（初期値: 5.0））
      jnt_speed(float): speed (PTP motion, Joint motion)[%] Default:5.0 （速度(PTP動作、Joint動作、最適直線補間動作)[%]（初期値: 5.0））
      acctime(float): acceleration time[s] Default:0.4 （加速時間[s]（初期値: 0.4））
      dacctime(float): deceleration time[s] Default:0.4 （減速時間[s]（初期値: 0.4)）
      posture(int): posture parameter. Default:2　（姿勢（初期値: 2)）
      passm(int): pass motion parameter. Default:2 （パス動作（初期値: 2））
      overlap(float): overlap motion parameter. Default:0.0 （オーバーラップ距離[mm]（初期値：0.0））
      zone(int):positioning settling range. Default:100 （位置決め完了範囲[pulse]（初期値: 100））
      pose_speed(float): speed (posture interpolated motion)　Default:20.0 （速度[%]（姿勢補間動作）(初期値: 20.0））
      ik_solver_option(long): Each joint rotation value. Default:0x11111111 （回転方向[flag]（初期値: 0x11111111））

    Return:
      MotionParam: Reference to instance （MotionParam クラスオブジェクトへの参照）

    See Also:
      See the explanation of constructor as for each parameter.
      （各パラメータの意味は、コンストラクタの説明を参照して下さい。）

    **Example**::

      m=MotionParam()

      # Ex1: Omit parameter (Use Initial values)
      # 例1：引数省略( 初期値を設定する)
      mm=m.motionparam()

      # Ex2: Specify some parameters (Overwrite some parameters on default values)
      # 例2：動作パラメータを変更する
      mm=m.motionparam( lin_speed=70, jnt_speed=10, overlap=30 )

    """
    self.__param(*arg1, **arg2)
    return self

'''==========================================================================
[クラス]
  Area_box
  平行6面体の領域を定義する。

■ メソッド（関数）
  inarea（p）：点Pが平行6面体の領域内に入っているかどうかを判定する。
  戻り値
  True  : 内部の点
  False : 外部の点

  ただし、invがTrueの場合には、反転する。

■ メンバ変数
  視点は原点から平行6面体を見た方向とする。
  p1 : 平行6面体の底面の左端の頂点
  p2 : 平行6面体の底面の真中の頂点
  p3 : 平行6面体の底面の右端の頂点
  p4 : 平行6面体の上面の真中の頂点
 ==========================================================================='''
#[Public] --------------------------
class Area_box(object):
  u"""Check if a robot position is inside the parallelepiped area. （平行6面体の領域判定を行う）"""

  def __init__(self, p1, p2, p3, p4, inv = False):
    u"""instantiate object to define a checking area. （コンストラクタ。平行6面体の領域を定義するインスタンスを作成する）

    View parallelepiped from orign. （視点は原点から平行6面体を見た方向とする）

    Args:
      p1(Position): Left side point of bottom surface of the parallelepiped. （平行6面体の底面の左端の頂点）
      p2(Position): Center point of bottom surface of the parallelepiped. （平行6面体の底面の真中の頂点）
      p3(Position): Right side point of bottom surface of the parallelepiped. （平行6面体の底面の右端の頂点）
      p4(Position): Center point of top surface of the parallelepiped. （平行6面体の上面の真中の頂点）
      inv(bool): If true, result value is inverted. Default:False （Trueのとき、領域判定結果を反転する（初期値: False)）

    Return:
      Area_box: Reference to instance  （Area_boxクラスオブジェクトへの参照）

    **Example**::

      # Define a parallelepiped by four points.
      # 空間上の4点の座標から、平行6面体を定義する。（姿勢成分は使わない）
      p1 = Position(x1,y1,z1,0,0,0,_BASE)
      p2 = Position(x2,y2,z2,0,0,0,_BASE)
      p3 = Position(x3,y3,z3,0,0,0,_BASE)
      p4 = Position(x4,y4,z4,0,0,0,_BASE)
      area_obj = Area_box(p1,p2,p3,p4)

    """
    self.__p1  = p1.position()[0:3]
    self.__p2  = p2.position()[0:3]
    self.__p3  = p3.position()[0:3]
    self.__p4  = p4.position()[0:3]
    self.__inv = inv

    # 頂点2（ｐ２）を原点として稜線の方向を計算
    self.__sub21 = self.sub_vector(self.__p1, self.__p2)
    self.__sub23 = self.sub_vector(self.__p3, self.__p2)
    self.__sub24 = self.sub_vector(self.__p4, self.__p2)

    # 稜線の長さを計算
    self.__dist21 = self.calc_distance(self.__p1, self.__p2)
    self.__dist23 = self.calc_distance(self.__p3, self.__p2)
    self.__dist24 = self.calc_distance(self.__p4, self.__p2)
    self.__det = None
    self.__inva = None

    if self.__dist21 == 0.0 or self.__dist23 == 0.0 or self.__dist24 == 0.0:
        liberr(4,16,"Area_box.init")

    # 逆行列の計算の準備
    a =  [[0 for _ in range(3)] for _ in range(3)]
    self.__inva = [[0 for _ in range(3)] for _ in range(3)]

    # 行列の設定
    a[0][0] = self.__sub21[0]
    a[0][1] = self.__sub23[0]
    a[0][2] = self.__sub24[0]

    a[1][0] = self.__sub21[1]
    a[1][1] = self.__sub23[1]
    a[1][2] = self.__sub24[1]

    a[2][0] = self.__sub21[2]
    a[2][1] = self.__sub23[2]
    a[2][2] = self.__sub24[2]

    # 逆行列の計算
    #pylint: disable=W0633
    self.__det, self.__inva = self.inv_matrix33(a)


  # 2点間の距離を計算する
  def calc_distance(self, point1, point2):
    u"""Private (非公開)"""
    dist = (point1[0] -point2[0])**2 +(point1[1] -point2[1])**2 \
          +(point1[2] -point2[2])**2
    dist = math.sqrt(dist)
    return dist

  # 2点を結ぶベクトルを計算する
  def sub_vector(self, point1, point2):
    u"""Private (非公開)"""
    sub = []
    sub.append(0)
    sub.append(1)
    sub.append(2)
    sub[0] = point1[0] -point2[0]
    sub[1] = point1[1] -point2[1]
    sub[2] = point1[2] -point2[2]

    return sub

  # 3x3行列の逆行列を計算する。
  def inv_matrix33(self, a):
    u"""Private (非公開)"""
    # 逆行列用配列の宣言
    inv_a =  [[0 for _ in range(3)] for _ in range(3)]

    # 行列の設定
    a11 = a[0][0]
    a12 = a[0][1]
    a13 = a[0][2]

    a21 = a[1][0]
    a22 = a[1][1]
    a23 = a[1][2]

    a31 = a[2][0]
    a32 = a[2][1]
    a33 = a[2][2]

    # 3x3行列式の計算
    det = a11*a22*a33 +a21*a32*a13 +a31*a12*a23 \
         -a11*a32*a23 -a31*a22*a13 -a21*a12*a33

    # 3x3逆行列の成分の計算
    if det != 0.0:
        inv_a[0][0] = a22*a33 -a23*a32
        inv_a[0][1] = a13*a32 -a12*a33
        inv_a[0][2] = a12*a23 -a13*a22

        inv_a[1][0] = a23*a31 -a21*a33
        inv_a[1][1] = a11*a33 -a13*a31
        inv_a[1][2] = a13*a21 -a11*a23

        inv_a[2][0] = a21*a32 -a22*a31
        inv_a[2][1] = a12*a31 -a11*a32
        inv_a[2][2] = a11*a22 -a12*a21
    else:
        return liberr(4,20,"Area_box.inv_matrix33")

    return det, inv_a

  # 点が領域の内部にあるか調べる。
  def inarea(self, p):
    u"""check if the specified point is inside or not. （指定した座標が領域の内部にあるか調べる）

    Args:
      p(Position): Target position （調べる対象となる座標）

    Return:
      bool: whether the point is inside or not. （領域の内部にあるかどうか）

        * True: Inside （領域内）
        * False: Outside （領域外）

    Note:
      * Return value is inverted if inv = False in constructor. （コンストラクタで、inv=Trueを指定した場合は、戻り値の値が反転します。）

    **Example**::

      # p is a target position
      # 調べる対象となる座標をp変数に代入しておく
      if area_obj.inarea(p):
        # process if p is inside...
        # 領域内のときの処理
      else:
        # process if p is outside...
        # 領域外のときの処理

    """
    p0 = p.position()[0:3]
    #点2を原点とした調べる点の位置ベクトル
    subp  = self.sub_vector(p0, self.__p2)

    # 点2を原点とした調べる点に、3つの稜線ベクトルから構成される行列の逆行列を乗算して、
    # 各稜線方向への成分を計算する。
    comp = [0 for i in range(3)]

    for i in range(3):
        sums = 0.0
        for k in range(3):
            sums += self.__inva[i][k]*subp[k]
        comp[i] = sums/self.__det

    # 点が平行6面体内にあるかの確認
    result = True

    if comp[0] < 0.0 or comp[0] > 1.0:
        result = False

    if comp[1] < 0.0 or comp[1] > 1.0:
        result = False

    if comp[2] < 0.0 or comp[2] > 1.0:
        result = False

    # 調べる点が領域を指定する場合には領域内の点とする
    if (p0 == self.__p1 or p0 == self.__p2 or p0 == self.__p3
       or p0 == self.__p4):
        result = True

    #点が外部にあることを判定する場合(inv = True)
    if self.__inv == True:
        result = not result
    return result

'''==========================================================================
[クラス]
  Area_cube
  直方体の領域を定義する。

■ メソッド（関数）
  inarea（p）：点Pが平行6面体の領域内に入っているかどうかを判定する。
  戻り値
  True  : 内部の点
  False : 外部の点

  ただし、invがTrueの場合には、反転する。

■ メンバ変数
  直方体の対角線で向き合う頂点の対を指定することで、直方体の領域を指定する。
  p1 : 対角線で向き合う頂点の対の一つ
  p2 : 対角線で向き合う頂点の対の一つ
  注》　p1とp2が同じ点の場合は、立方体は点となる。
 ==========================================================================='''
#[Public] --------------------------
class Area_cube(object):
    u"""Check if a robot position is inside the rectangular area. （直方体の領域判定を行う）"""

    def __init__(self, p1, p2, inv = False):
        u"""instantiate object to define a checking area. （コンストラクタ。直方体の領域を定義するインスタンスを作成する）

        Define rectangular by point pair of diagonal. Each side is parallel with each axis.
        （直方体の対角線で向き合う頂点の対を指定することで、直方体の領域を指定する。なお、直方体の
        各辺はそれぞれXYZ軸のいずれかに平行となる。）

        Args:
          p1(Position): one side of diagonal （対角線で向き合う頂点の対の一つ）
          p2(Position): the other side of diagonal （対角線で向き合う頂点の対の一つ）
          inv(bool): If true, result value is inverted. Default:False （Trueのとき、領域判定結果を反転する（初期値: False)）

        Return:
          Area_cube: Reference to instance  （Area_cubeクラスオブジェクトへの参照）

        **Example**::

          # Define a rectangular by two points.
          # 空間上の2点の座標から、直方体を定義する。（姿勢成分は使わない）
          p1 = Position(x1,y1,z1,0,0,0,_BASE)
          p2 = Position(x2,y2,z2,0,0,0,_BASE)
          area_obj = Area_cube(p1,p2)

        """
        self.__p1  = p1.position()[0:3]
        self.__p2  = p2.position()[0:3]
        self.__inv = inv

    # ベクトルの差を計算する
    def sub_vector(self, point1, point2):
        u"""Private (非公開)"""
        sub = []
        sub.append(0)
        sub.append(1)
        sub.append(2)
        sub[0] = point1[0] -point2[0]
        sub[1] = point1[1] -point2[1]
        sub[2] = point1[2] -point2[2]

        return sub

    # 与えた点が直方体の内部にあるか調べる。
    def inarea(self, p):
        u"""check if the specified point is inside or not. （指定した座標が領域の内部にあるか調べる）

        Args:
          p(Position): Target position （調べる対象となる座標）

        Return:
          bool: whether the point is inside or not. （領域の内部にあるかどうか）

            * True: Inside （領域内）
            * False: Outside （領域外）

        Note:
          * Return value is inverted if inv = False in constructor. （コンストラクタで、inv=Trueを指定した場合は、戻り値の値が反転します。）

        **Example**::

          # p is a target position
          # 調べる対象となる座標をp変数に代入しておく
          if area_obj.inarea(p):
            # process if p is inside...
            # 領域内のときの処理
          else:
            # process if p is outside...
            # 領域外のときの処理

        """
        p0 = p.position()[0:3]
        #p2の位置ベクトルからp1の位置ベクトルを引くことで、直方体の(縦, 横, 高さ)を計算する
        scale = []
        scale.append(0)
        scale.append(1)
        scale.append(2)

        scale = self.sub_vector(self.__p2, self.__p1)

        #縦、横、高さの符号を調べる。p1を原点とした系に、この符号を乗算する。
        sign_x = 1
        sign_y = 1
        sign_z = 1

        if scale[0] < 0:
            sign_x = -1
        if scale[1] < 0:
            sign_y = -1
        if scale[2] < 0:
            sign_z = -1

        #p1を原点としたときの、点pの位置ベクトルを計算する。
        p3 = self.sub_vector(p0, self.__p1)

        #pの各座標に符号を乗算する。
        p3[0] *= sign_x
        p3[1] *= sign_y
        p3[2] *= sign_z

        #　直方体の大きさも絶対値にする
        scale[0] *= sign_x
        scale[1] *= sign_y
        scale[2] *= sign_z

        #点pが直方体の内部にあるか調べる。
        result = True

        if p3[0] < 0.0 or p3[0] > scale[0]:
            result = False

        if p3[1] < 0.0 or p3[1] > scale[1]:
            result = False

        if p3[2] < 0.0 or p3[2] > scale[2]:
            result = False

        #点が外部にあることを判定する場合(inv = True)
        if self.__inv == True:
            result = not result

        return result

'''============================================================================
[クラス]
  Area_cyl
  両端が半球の円柱状の領域を定義する。

■ メソッド（関数）
  inarea（p）：点Pが円柱状の領域内に入っているかどうかを判定する。
  戻り値
  True  : 内部の点
  False : 外部の点

  ただし、invがTrueの場合には、反転する。

■ メンバ変数
  p1 : 円柱状の領域の端の点
  p2 : 円柱状の領域の端の点
  R　 : 両端の球状の領域の半径
============================================================================'''
#[Public] --------------------------
class Area_cyl(object):
    u"""Check if a robot position is inside the capsule-shaped area. （カプセル形状の領域判定を行う）"""

    def __init__(self, p1, p2, R, inv = False):
        u"""instantiate object to define a checking area. コンストラクタ。カプセル形状の領域を定義するインスタンスを作成する

        Capsule-shaped means hemispheres are attached both sides of Cylinder. （カプセル形状とは、両端が半球の円柱状となる。）

        Args:
          p1(Position): center point of one hemispheres. （一方の半球の中心）
          p2(Position): center point of the other hemispheres. （もう一方の半球の中心）
          R(float): radius of hemispheres （半球の半径）
          inv(bool): If true, result value is inverted. Default:False （Trueのとき、領域判定結果を反転する（初期値: False)）

        Return:
          Area_cyl: Reference to instance （Area_cyl クラスオブジェクトへの参照）

        **Example**::

          # Define a capsule-shaped area by 2 points and radius.
          # 空間上の2点の座標と半径から、カプセル形状の領域を定義する。（姿勢成分は使わない）
          p1 = Position(x1,y1,z1,0,0,0,_BASE)
          p2 = Position(x2,y2,z2,0,0,0,_BASE)
          r  = 100.0
          area_obj = Area_cyl(p1,p2,r)

        """
        self.__p1  = p1.position()[0:3]
        self.__p2  = p2.position()[0:3]
        self.__R   = R
        self.__inv = inv

    # 2点を結ぶベクトルを計算する
    def sub_vector(self, point1, point2):
        u"""Private (非公開)"""
        sub = []
        sub.append(0)
        sub.append(1)
        sub.append(2)
        sub[0] = point1[0] -point2[0]
        sub[1] = point1[1] -point2[1]
        sub[2] = point1[2] -point2[2]

        return sub

    # ベクトルの内積を計算する
    def inner_product(self, vector1, vector2):
        u"""Private (非公開)"""
        product = vector1[0]*vector2[0] +vector1[1]*vector2[1] \
                 +vector1[2]*vector2[2]
        return product

    # 点が内部にあるか調べる
    def inarea(self, p):
        u"""check if the specified point is inside or not. （指定した座標が領域の内部にあるか調べる）

        Args:
          p(Position): Target position （調べる対象となる座標）

        Return:
          bool: whether the point is inside or not. （領域の内部にあるかどうか）

            * True: Inside （領域内）
            * False: Outside （領域外）

        Note:
          * Return value is inverted if inv = False in constructor. （コンストラクタで、inv=Trueを指定した場合は、戻り値の値が反転します。）

        **Example**::

          # p is a target position
          # 調べる対象となる座標をp変数に代入しておく
          if area_obj.inarea(p):
            # process if p is inside...
            # 領域内のときの処理
          else:
            # process if p is outside...
            # 領域外のときの処理

        """
        p0 = p.position()[0:3]
        #p2を原点とした位置ベクトルを求める。
        subp = self.sub_vector(p0, self.__p2)
        sub12 = self.sub_vector(self.__p1, self.__p2)

        # p2とpの距離の2乗を求める。
        distp2 = self.inner_product(subp, subp)

        # p2->p１間の距離の2乗を求める。
        dist12 = self.inner_product(sub12, sub12)

        # pの垂線の足を求める
        # pの位置ベクトルと2->1方向のベクトルの内積を計算する。
        xp = self.inner_product(subp, sub12)

        if dist12 > 0.0:
            xp = xp/math.sqrt(dist12)
        else:
            xp = 0

        # 垂線の足とpの距離を計算する。
        dist = math.sqrt(distp2 -xp**2)

        # 点が円筒の内部にあるか調べる。
        result = True

        if xp > 0.0 and xp < math.sqrt(dist12):
            if dist > self.__R:
                result = False
        if xp < 0.0:
            dist = math.sqrt(distp2)
            if dist > self.__R:
                result = False
        if xp >= math.sqrt(dist12):
            subp1 = self.sub_vector(p0, self.__p1)
            distp1 = self.inner_product(subp1, subp1)
            distp1 = math.sqrt(distp1)
            if distp1 > self.__R:
                result = False

        #点が外部にあることを判定する場合(inv = True)
        if self.__inv == True:
            result = not result

        return result
"""======================================================================
[クラス]
  i611Robotクラス : ロボット動作ラッパー

■ メソッド(関数)
__init__(self, host= "127.0.0.1", port=12345, _enableError = True):	
def __del__(self):	
version(self):	i611nativeのバージョン
MCS_version(self):	i611pythonライブラリのバージョン
open(self):	i611nativeとの接続開始、初期化
close(self):	i611nativeとの接続終了、終了処理
exit(self):	強制終了
svoff(self):	サーボOFF
asyncm(self, sw):	非同期動作 ON/OFF
abort(self):	ロボット動作の中断
join(self):	実行中の動作コマンドキュー完了待ち
home(self):	各軸all=0deg位置へ移動
move(self, *cmd):	PTP動作
line(self, *cmd):	直線補間動作
optline(self, *cmd):最適直線補間動作
toolmove(self, *arg1, **arg2):	ツール座標系相対動作
motionparam(self, *arg1, **arg2):	動作条件の設定
getmotionparam(self):	設定中の動作条件
override(self, ovr):	オーバーライド
set_restrict(self, *area):	動作制限範囲の設定
areacheck(self, pos):	動作制限範囲の内外判定
settool(self, *arg1, **arg2):	ツールオフセットの設定
changetool(self, id):	ツールオフセットの選択
set_mdo(self, mdoid, portno, value, kind, distance):	MDO動作の設定
enable_mdo(self, bitfield):	MDO動作有効化
disable_mdo(self, bitfield):	MDO動作無効化
getpos(self):	現在値(ワールド座標系)を取得
getjnt(self):	現在値(JNT座標系)を取得
Joint2Position(self, *jnt):	JNT座標系からワールド座標系への変換
Position2Joint(self, *pos):	ワールド座標系からJNT座標系への変換

■　内部メソッド(関数)
_error(self, src2, div, code):	エラーログ
_mcslog(self, src2, *arg):	コマンド実行ログ
_enableErr(self, sw = True):	エラーハンドリング ON/OFF
__ptpmove(self, pram):	PTP動作 サブ関数
__cpmove(self, pram):	直線補間動作 サブ関数
__optcpmove(self, pram):最適直線補間動作 サブ関数
__jntmove(self, pram):	関節動作 サブ関数
__chMotion(self, pram):	動作条件変更 サブ関数
_open(self):	i611native
_close(self):	i611native
_abortm(self):	i611native
_svctrl(self, sw):	i611native
_plsmove(self, ax1, ax2, ax3, ax4, ax5, ax6, speed, acct, dacct):	i611native
_mtrmove(self, ax1, ax2, ax3, ax4, ax5, ax6, speed, acct, dacct):	i611native
_jntmove(self, ax1, ax2, ax3, ax4, ax5, ax6, speed, acct, dacct):	i611native
_ptpmove(self, x, y, z, rz, ry, rx, posture, rbcoord, speed, acct, dacct):	i611native
_ptpmove_mt(self, x, y, z, rz, ry, rx, posture, rbcoord, multiturn, speed, acct, dacct):	i611native
_cpmove(self, x, y, z, rz, ry, rx, posture, rbcoord, speed, acct, dacct):	i611native
_optcpmove(self, x, y, z, rz, ry, rx, posture, rbcoord, speed, acct, dacct):i611native
_mark(self):	i611native
_mark_mt(self):	i611native
_jmark(self):	i611native
_pmark(self, sw):	i611native
_joinm(self):	i611native
_acq_permission(self):	i611native
_rel_permission(self):	i611native
_ioctrl(self, wordno, dataL, maskL, dataH, maskH):	i611native
  
■　内部変数
  __mpdef  :  動作パラメータ規定値
  __ovr  :  オーバーライド現在設定値
  __mp  :  動作パラメータ現在設定値
  __mpdef  :  規定動作パラメータ
  __mp2  :  __mp*__ovr
======================================================================"""
#[Internal]  --------------------------
g_i611RobotInstance = None #:(非公開)
#[Internal] --------------------------
def has_i611Robot_instance():
  u"""Private (非公開)"""
  if g_i611RobotInstance:
    return True
  else:
    return False

class i611Robot(object):
  u"""Defines robot motions （ロボットの動作を扱う）"""

  #[Public] ############################
  def MCS_version(self):
    u"""Gets the version information of the python robot library. （ロボットライブラリのバージョンを取得する）

    Args:
      None

    Return:
      list: [major, minor, patch, build]

        - major(int): Major version （メジャーバージョン）
        - minor(int): Minor version （マイナーバージョン）
        - patch(int): Patch version （パッチバージョン）
        - build(int): Build version （ビルドバージョン）

    **Example**::

      rb.MCS_version()

    """
    return version_i611_MCS()

  #[Public] ############################
  def version(self):
    u"""Obtains the system version information. （システムバージョンを取得する）

    Args:
      None

    Return:
      list: [res0, major, minor, patch, build, date, option]

        - res0(bool): True:Success, False:Failure （Trueのとき成功、Falseのとき失敗）
        - major(int): Major version （メジャーバージョン）
        - minor(int): Minor version （マイナーバージョン）
        - patch(int): Patch version （パッチバージョン）
        - build(int): Build version （ビルドバージョン）
        - date(str): Build date （ビルド日付）
        - option(str): Option （オプション）

    **Example**::

      rb.version()

    """
    return self.__rblib.version()

  #[Internal] --------------------------
  def __init__(self, host= "127.0.0.1", port=12345, _enableError=True,
    _disableDin=False, _teach=False, _monitor=False):
    u"""Initiates the i611 class and calls the constructor. （コンストラクタ。i611 クラスのインスタンスを作成する）

    Args:
      host(str): Specify the host IP address. Default:'127.0.0.1'（接続先のIP アドレス（初期値：'127.0.0.1'））
      port(int): Specify the host post number. Default:12345 （接続先のポート番号（初期値：12345））
      * The other options are reserved. （その他のオプションはシステム予約となります。）

    Return:
            i611Robot: Reference to instance  （i611Robotクラスオブジェクトへの参照）

    **Example**::

      # Ex: Omit parameters ( use Default parameters )
      # 例：引数省略( 初期値を設定する)
      rb=i611Robot()


    """
    liblog("i611Robot.init",_teach,_monitor)
##    sys.setcheckinterval(10)
    self.__isopened             = False
    self.__mlog                 = []
    self.__ovr                  = 1.0
    self.__restrict             = []
    self._moveing               = False
    self.accel_limit            = False   # [ACCL] 加速抑制On/Off
    self.min_ticks              = 160.0   # [ACCL] 160ms (default)

    if bool(_monitor):
      _teach = True

    if bool(_teach):
      _enableError = False
      _disableDin = True

    self.__mpdef                 = MotionParam().mp2list()
    self.__mp                    = self.__mpdef
    self.__mp2                   = self.__mpdef
    self.__rblib                 = rblib.Robot(host, port)
    self.__sysstatus             = _WatchStatus(host, port, self)
    self.__sysstatus._enbErr     = bool(_enableError)
    self.__sysstatus._disableDin = bool(_disableDin)
    self.__sysstatus._teachMode  = bool(_teach)
    self.__sysstatus._monitorMode = bool(_monitor)
    self._host                   = host
    self._port                   = port
    self._sys_exit_code = 0
    self._sys_exception = None
    self._org_excepthook = None
    self._org_exit = None
    self.last_exit_code = 0
    self.api_thread_id = thread.get_ident()
    self.last_pos = []
    self.last_jnt = []
    self.teachdata_ver = Teachdata.version()
    self.first_parameter_update = True

    # check if current mode is error
    self.__rblib.open()
    ret = self.__rblib.ioctrl(130, 0, 0xffffffff, 0, 0xffffffff)
    if ret[0] == False:
      self.__cause_fatal_exception(18)
    st = ( ret[1] >> 4 ) & 0x0F
    if st>=10:
      liberr(4,21)
    self.__rblib.close()
    if self.__sysstatus._teachMode is False and not os.path.exists("/tmp/auto_ready"):
      liberr(4,22)

    global ONCE_CREATED
    ONCE_CREATED = True # フラグを立てるのは、WatchStatus()を作ってから
    global g_i611RobotInstance
    g_i611RobotInstance = self

    if not self.__sysstatus._teachMode:
      self.__enable_hook()
      #pylint: disable=maybe-no-member
      signal.signal(signal.SIGALRM, self.emo_handler)
      ret = self.__sysstatus.register_program()
      if ret[0] == False:
        liberr(4,23)

      ## reflect app status
      self.__write_app_status(0,0)

  #[Internal] --------------------------
  def __del__(self):
    u"""デストラクタ"""
    liblog("i611Robot.del")
    self.close()

  #[Internal] --------------------------
  def __write_app_status(self,status,code,pause_status=0):
    u"""Private (非公開)"""
    if self.__sysstatus._teachMode:
      return
    st_app_status = 0
    if status>-1:
      st_app_status = ((status&0x0F) << 24)
    if code>-1:
      st_app_status |= (code&0x0FF) << 16
    if self.__sysstatus._disableDin:
        st_app_status |= 0x10000000
    st_app_status |= (pause_status&0x07) << 29
    self.__sysstatus._cur_pause_status = pause_status
    if not self.__isopened:
      self.__rblib.open()
    ret = self.__rblib.ioctrl(130, st_app_status, 0x0000ffff, 0, 0xffffffff)
    if ret[0] == False:
      self.__cause_fatal_exception(18)
    if not self.__isopened:
      self.__rblib.close()
    # verify
    status, code = self.__read_app_status()
#    print "__write_app_status(%d,%d) verify end" % (status, code)


  #[Internal] --------------------------
  def __read_app_status(self):
    u"""Private (非公開)"""
    if not self.__isopened:
      self.__rblib.open()
    ret = self.__rblib.ioctrl(130, 0, 0xffffffff, 0, 0xffffffff)
    if ret[0] == False:
      self.__cause_fatal_exception(18)
    value = ret[1]
    if not self.__isopened:
      self.__rblib.close()
    status = (value >> 24) & 0x0F
    code = (value >> 16) & 0x0FF
    return (status,code)

  #[Internal] --------------------------
  def __cause_fatal_exit(self, code, st=2):
    u"""Private (非公開)"""
    liblog("i611Robot.cause_fatal_exit",code,st)
    if self.__sysstatus._teachMode:
      return
    if self.api_thread_id != thread.get_ident():
      self.__sysstatus._fatal_error = self.__sysstatus.EID_INVALIDTHREAD
      self.__sysstatus._ev_fatal_error.set()
    else:
      print "cause fatal exit(%d:%s)" % (code, self._get_syserr_msg(10,code))
      self.__write_app_status(st,code)
      os._exit(1)

  #[Internal] --------------------------
  def __cause_fatal_exception(self, code):
    u"""Private (非公開)"""
    liblog("i611Robot.cause_fatal_exception",code)
    if self.__sysstatus._teachMode:
      return
    if self.api_thread_id != thread.get_ident():
      self.__sysstatus._fatal_error = self.__sysstatus.EID_INVALIDTHREAD
      self.__sysstatus._ev_fatal_error.set()
    else:
      self.last_exit_code = code
      self.__write_app_status(3,self.last_exit_code) # write immediately
      print "cause fatal exception!(%d)" % code
      self.__sysstatus._ev_fatal_error.set()
      errmsg = "(code=%d:%s)" % (code,self._get_syserr_msg(10,code))
      raise Robot_fatalerror(errmsg)

  #[Internal] --------------------------
  def __enable_hook(self):
    u"""Private (非公開)"""
    if self.__sysstatus._teachMode:
      return
    self._org_exit = sys.exit
    self._org_excepthook = sys.excepthook
    sys.exit = self.__custom_exit
    sys.excepthook = self.__custom_excephook
    atexit.register(self._hook_atexit)

  #[Internal] --------------------------
  def __custom_excephook(self,exc_type, exc, *args):
    u"""Private (非公開)"""
    print "__custom_excephook(last_exit=%d)" % self.last_exit_code, exc
    self._sys_exception = exc
    if self.last_exit_code == 0:
        self.last_exit_code = 15  # NERR(15)
    _,cur_code = self.__read_app_status()
    if ONCE_CREATED is False or cur_code == 0: # Don't overwrite
      self.__write_app_status(3,self.last_exit_code)
    self._org_excepthook(exc_type, exc, *args)

  #[Internal] --------------------------
  def __custom_exit(self, code=0):
    u"""Private (非公開)"""
    liblog("i611Robot.custom_exit",code)
#    print "__custom_exit(%d)" % code
    self._sys_exit_code = code
    if code!=0:
      self.__write_app_status(2,14) # NERR(14)
    self._org_exit(code)

  #[Internal] --------------------------
  def _hook_atexit(self):
    u"""Private (非公開)"""
    self.__sysstatus.thread_end()
    if self.__sysstatus._teachMode:
      return
#    print "app_status=", self.__read_app_status()
    if self._sys_exception is not None: #exception exit
#      print "_hook_atexit by exception"
      pass
    elif self._sys_exit_code != 0: # error exit
#      print "_hook_atexit by error exit"
      pass
    else: # normal exit
      status,_ = self.__read_app_status()
#      print "_hook_atexit (st=%d,code=%d)" % (status, code)
      if status == 0:
        self.__write_app_status(1,0)

  #[Internal] --------------------------
  # call after 5 seconds from EMO
  def emo_handler(self, signum, frame):
    u"""Private (非公開)"""
    liblog("i611Robot.emo_handler,timeout")
    print "EMO timeout"
    _ = signum
    _ = frame
    self.__cause_fatal_exit(17)

  #[Public] ############################
  def open(self, permission=True):
    u"""Starts connecting with the robot and processes initialization. （ロボットとの接続を開始する（初期化をする））

    Args:
      permission(bool): Must Specify True （Trueを指定してください。（初期値: True)）

    Return:
      bool: Returns True if it succeed, otherwise an exception is raised. （成功したときにTrueが返り、それ以外は例外が発生します。）

    **Example**::

      rb.open(True)

    """
    #print "D:open start"
    if self.__isopened == False:
      liblog("i611Robot.open")

      if self.api_thread_id != thread.get_ident():
        print "open api must be called in main thread"
        self.__sysstatus._fatal_error = self.__sysstatus.EID_INVALIDTHREAD
        self.__sysstatus._ev_fatal_error.set()
        return

      global ONCE_OPENED
      if ONCE_OPENED:
        self.__cause_fatal_exception(20)
      ONCE_OPENED = True

      try:
        res = self.__rblib.open()
        self.__isopened = True
      except socket.error:
        return self._error(4, 6)

      if self.__sysstatus._enbErr == False:
        liberr_config(log_level=2, err_level=5)
        res = self.__rblib.set_log_level(2)
        if res[0] == False:
          return self._error(res[1], res[2])
      else:
        res = self.__rblib.set_log_level(1)
        if res[0] == False:
          return self._error(res[1], res[2])

      if permission:
        res = self.__rblib.acq_permission()
        if res[0] == False:
            return self.__cause_fatal_exception(11)  # NERR(11)

        if not self.__sysstatus._teachMode:
          if self.svstat() == -1:
            return self.__cause_fatal_exception(9)  # NERR(9)
          if self.svstat() != 1:
            return self.__cause_fatal_exception(10)  # NERR(10)

        res = self.__rblib.changetool(0)
        if res[0] == False:
          return self._error(res[1], res[2])

        res = self.__rblib.asyncm(2)
        if res[0] == False:
          return self._error(res[1], res[2])

        self.__sysstatus.start() # start only with permission

      return [True]
    else:
      return [False]

  #[Public] ############################
  def close(self):
    u"""Performs a close process which will terminate the connection to the robot. （ロボットとの接続を終了する）

    Args:
      None

    Return:
      bool: 成功したときにTrueが返り、それ以外は例外が発生します。
      bool: Returns True if it succeed, otherwise an exception is raised. （成功したときにTrueが返り、それ以外は例外が発生します。）

    **Example**::

      rb.close()

    """
    if self.__isopened == True:
      liblog("i611Robot.close")
      self.__sysstatus.thread_end()
      self.__rblib.rel_permission()
      self.__rblib.close()
      self.__isopened = False
      return [True]

  #[Public] ############################
  def is_open(self):
    u"""Checks if i611Robot is open. （i611Robot のオープン状態を確認する）

    Args:
      None

    Return:
      bool: The state of the instance (of type boolean)（オープン状態）

        * True: being opened（オープン中）
        * False: not opened（オープンされていない）

    **Example**::

      ## Watching in another thread
      ## 別スレッドで一時停止、再開の状態を監視する。
      def thread_fnc(rb):
      while not thread_end:
        # Check status （状態を確認）
        pause_st = rb.is_pause()
        print 'This status is {}.'.format(pause_st)
        print "th:wait stop",din(DIN_STOP)
        if din(DIN_STOP) == "1":
        rb.stop()
        if din(DIN_PAUSE) == "1":
          rb.pause()
        if din(DIN_RESTART) == "1":
          rb.restart()

      # Ex) Robot control sample （ロボットプログラムサンプル）
      try:
        while True:
        # motion command like line() or move()... （・・line(),move() などの動作プログラム記述・・）
        # program is paused in user_hook(). （user hook で一時停止）
        rb.user_hook()
        # motion command like line() or move()... （・・line(),move() などの動作プログラム記述・・）
      except Robot_emo: # catch Emergency sw event （非常停止SW 押下イベントハンドラ）
        # ・・・・
      except Robot_stop: # catch stop event （減速停止入力検知イベントハンドラ）
        # ・・・・
      finally:
        rb.close()

    """
    return self.__isopened

  #[Public] ############################
  def is_opened(self):
    u"""is_open()と同じ"""
    return self.__isopened

  #[Public] ############################
  def exit(self, res=0):
    u"""Aborts a robot program. （ロボットプログラムを強制終了する）

    Args:
      res(int): Exit code （終了コード）

        * 0: Normal exit （正常終了）
        * Not 0: Abnormal exit （異常終了）

    Return:
      None

    Note:
      * Don't need to call this method if a program exit normally. （ロボットプログラムを正常に終えるときはこのメソッドは必要ありません。）

        - close() is called in exit(). （exit() 処理ではclose() 処理も行われます。）

      * When exit code is not 0, system status become E14. 引数に０以外を指定してロボットプログラムを終了した場合、コントローラはシステム定義エラーE14になり、ロボットプログラムは異常終了します。

    **Example**::

      rb.exit( 0 )

    """
    liblog("i611Robot.exit")
    self.close()
    sys.exit(res)

  #[Public] ############################
  def svoff(self):
    u"""Turns the servo off. （サーボをOFF にする）

    Args:
      None

    Return:
      bool: Returns True if it succeed, otherwise an exception is raised. （成功したときにTrueが返り、それ以外は例外が発生します。）

    **Example**::

      rb.svoff()

    """
    ## (no hook) self._internal_hook() ###[HOOK]###
#    self._mcslog(14)
    liblog("i611Robot.svoff")
    res = self.__rblib.svctrl(2)
    if res[0] == False:
      return self._error(res[1], res[2])

  #[Public] ############################
  # return -1:emo, 0:servo off, 1:servo on
  def svstat(self):
    u"""Obtains the Servo power state （サーボ状態を取得する）

    Args:
      None

    Return:
      int: Servo status （サーボ状態）

        * 1: Servo ON　（サーボON）
        * 0: Servo OFF （サーボOFF）
        * -1: Emergency stop state （非常停止中）

    **Example**::

      if rb.svstat() == 1: # Servo ON　（サーボON）
        ...
      elif rb.svstat() == 0: # Servo OFF （サーボOFF）
        ...
      elif rb.svstat() == -1: # Emergency stop state （非常停止中）
        ...

    """
    ## (no hook) self._internal_hook() ###[HOOK]###
    res = self.ioctrl(128, 0, 0xffffffff, 0, 0xffffffff)
    if res[0] == False:
      return self._error(res[1], res[2])
    if i611_common._bitflag(res[1], 1):
      return -1
    if i611_common._bitflag(res[1], 0):
      return 1
    return 0

  #[Public] ############################
  def asyncm(self, sw):
    u"""Controls ON/OFF of program prefetching. Sets up the data prefetch segment by prefetching, which
enables overlap motion. （ロボットプログラムの先読み動作区間を設定する）

    Args:
      sw(int): program prefetching setting

        * 1: program prefetching ON （プログラム先読み動作ON）
        * 2: program prefetching OFF (default) （プログラム先読み動作OFF ( 初期値)）

    Return:
      bool: Returns True if it succeed, otherwise an exception is raised. （成功したときにTrueが返り、それ以外は例外が発生します。）

    **Example**::

      rb.line(p10) # line motion to p10. （ティーチングポイントp10 へ直線補間移動）
      rb.asyncm(sw=1) # set prefetching ON （プログラム先読み動作ON　（rb.asyncm(1) でも可能））
      rb.line(p20,p21) # line motion to p20, followed by p21. （ティーチングポイントp20 とp21へ順に直線補間動作で移動する）
      rb.join() # end prefetching motion. （先読みしたロボットプログラムの完了を待機する）
      rb.asyncm(sw=2) # set prefetching OFF （プログラム先読み動作OFF　（rb.asyncm(2) でも可能））
      ...
      rb.close()

    """
    self._internal_hook() ###[HOOK]###
#    self._mcslog(15, sw)
    if not self.__sysstatus._teachMode:
      liblog("i611Robot.asyncm",sw)
    res = i611_common._chkparam(sw, p_type = int, min = 1, max = 2)
    if res[0] == False:
      return self._error(4, res[1])
    res = self.__rblib.asyncm(sw)
    if res[0] == False:
      return self._error(res[1], res[2])
    return [True]

  #[Public] ############################
  def abort(self):
    u"""Aborts the robot motion （動作中のロボットを減速停止する）

    Args:
      None

    Return:
      None

    Note:
      * It doesn't make sense when robot doesn't moved. （ロボット動作中以外は効果がありません）

    **Example**::

      rb.abort()

    """
    ret = self.__sysstatus.abort()
    liblog("%d = i611Robot.abort" % ret[1])

  #[Public] ############################
  def stop(self):
    u"""Emit stop signal （ロボットを減速停止する）

    Args:
      None

    Return:
      None

    Note:
      * This signal is the same as the input signal of Stop. （外部入力によるロボット停止信号を受けた時と同じ処理になります。）

    **Example**::

      ## emit stop in another thread
      ## 別スレッドで減速停止をさせる
      def thread_fnc(rb): # called in another thread. （別スレッド用関数）
        while not thread_end:
          pause_st = rb.is_pause()
          print 'This status is {}.'.format(pause_st)
          print "th:wait stop",din(DIN_STOP)

          #　call stop "減速停止"
          if din(DIN_STOP) == "1":
            rb.stop()
          if din(DIN_PAUSE) == "1":
            rb.pause()
          if din(DIN_RESTART) == "1":
            rb.restart()

      # Ex) robot program sample for main thread
      # 例）ロボットプログラムサンプル（メインスレッド）
      try:
        while True:

        # motion command like line() or move()... （・・line(),move() などの動作プログラム記述・・）
        # program is paused in user_hook(). （user hook で一時停止）
        rb.user_hook()
        # motion command like line() or move()... （・・line(),move() などの動作プログラム記述・・）

      except Robot_emo: # catch Emergency sw event （非常停止SW 押下イベントハンドラ）
        # ・・・・
      except Robot_stop: # catch stop event （減速停止入力検知イベントハンドラ）
        # ・・・・
      finally:
        rb.close()

    """
    liblog("i611Robot.stop")
    if not self.__sysstatus._ev_stop.is_set():
          self.__sysstatus._ev_stop.set()    # Stop Exception
          self.abort()
          # clear other events
          self.__sysstatus._ev_pause.clear()
          self.__sysstatus._ev_continue.clear()

  #[Public] ############################
  def pause(self):
    u"""Emit pause signal （ロボット動作を一時停止する）

    Args:
      None

    Return:
      None

    Note:
      * This signal is the same as the input signal of Pause. （外部入力によるロボット一時停止信号を受けた時と同じ処理になります。）

    **Example**::

      ## Watch pause status in another thread.
      ## 別スレッドで一時停止の状態を監視する。
      def thread_fnc(rb):
        while not thread_end:
          pause_st = rb.is_pause()
          print 'This status is {}.'.format(pause_st)
          print "th:wait stop",din(DIN_STOP)
          if din(DIN_STOP) == "1":
            rb.stop()
          if din(DIN_PAUSE) == "1":
      　　  # paused （一時停止させる）
            rb.pause()
          if din(DIN_RESTART) == "1":
            rb.restart()

      # Ex) Robot program sample
      # 例）ロボットプログラムサンプル
        try:
          while True:
            # motion command like line() or move()... （・・line(),move() などの動作プログラム記述・・）
            # program is paused in user_hook(). （user hook で一時停止）
            rb.user_hook()
            # motion command like line() or move()... （・・line(),move() などの動作プログラム記述・・）

        except Robot_emo: # catch Emergency sw event （非常停止SW 押下イベントハンドラ）
          # ・・・・
        except Robot_stop: # catch stop event （減速停止入力検知イベントハンドラ）
          # ・・・・
        finally:
          rb.close()

    """
    liblog("i611Robot.pause")
    if not self.__sysstatus._ev_pause.is_set():
      self.__sysstatus._ev_pause.set()
      # clear other events
      self.__sysstatus._ev_stop.clear()
      self.__sysstatus._ev_continue.clear()

  #[Public] ############################
  def restart(self):
    u"""Emit restart signal. 一時停止から動作を再開する

    Args:
      None

    Return:
      None

    Note:
      * This signal is the same as the input signal of Restart(Run). （外部入力によるロボット再開(run)信号を受けた時と同じ処理になります。）

    **Example**::

      ## emit restart in another thread
      ## 別スレッドで再開させる。
      def thread_fnc(rb):
        while not thread_end:
          pause_st = rb.is_pause()
          print 'This status is {}.'.format(pause_st)
          print "th:wait stop",din(DIN_STOP)
          if din(DIN_STOP) == "1":
            rb.stop()
          if din(DIN_PAUSE) == "1":
            rb.pause()
          if din(DIN_RESTART) == "1":
            # 再開させる
            rb.restart()

      # Ex) robot program sample
      # 例）ロボットプログラムサンプル
      try:
        while True:
          # motion command like line() or move()... （・・line(),move() などの動作プログラム記述・・）
          # program is paused in user_hook(). （user hook で一時停止）
          rb.user_hook()
          # motion command like line() or move()... （・・line(),move() などの動作プログラム記述・・）

      except Robot_emo: # catch Emergency sw event （非常停止SW 押下イベントハンドラ）
        # ・・・・
      except Robot_stop: # catch stop event （減速停止入力検知イベントハンドラ）
        # ・・・・
      finally:
        rb.close()


    """
    liblog("i611Robot.restart")
    self.__sysstatus._ev_continue.set()

  #[Public] ############################
  def is_pause(self):
    u"""Check if program is paused. （ロボットプログラムの一時停止中状態を確認する）

    Args:
      None

    Return:
      bool: Pause status (一時停止状態)

        * True: Now paused （一時停止中）
        * False: Not paused （一時停止中ではない）

    **Example**::

      ## Watch pause status in another thread
      ## 別スレッドで一時停止、再開の状態を監視する。
      def thread_fnc(rb):
        while not thread_end:
          # Check status （状態を確認）
          pause_st = rb.is_pause()
          print 'This status is {}.'.format(pause_st)
          print "th:wait stop",din(DIN_STOP)
          if din(DIN_STOP) == "1":
            rb.stop()
          if din(DIN_PAUSE) == "1":
            rb.pause()
          if din(DIN_RESTART) == "1":
            rb.restart()

      # Ex) robot program sample
      # 例）ロボットプログラムサンプル
      try:
        while True:
          # motion command like line() or move()... （・・line(),move() などの動作プログラム記述・・）
          # program is paused in user_hook(). （user hook で一時停止）
          rb.user_hook()
          # motion command like line() or move()... （・・line(),move() などの動作プログラム記述・・）

      except Robot_emo: # catch Emergency sw event （非常停止SW 押下イベントハンドラ）
        # ・・・・
      except Robot_stop: # catch stop event （減速停止入力検知イベントハンドラ）
        # ・・・・
      finally:
        rb.close()


    """
    val= int(shm_read(0x0308,1))
    in_pause = (val >> 2) & 0x01    ## 6:一時停止中
    if in_pause:
      return True
    else:
      return False

  #[Public] ############################
  def join(self):
    u"""Waits for the motion command queue being executed to complete.　（先読みしたロボットプログラムの完了を待機する）

    Args:
      None

    Return:
      bool: Returns True if it succeed, otherwise an exception is raised. （成功したときにTrueが返り、それ以外は例外が発生します。）

    **Example**::

      rb.line( P10 ) # line motion to P10. （ティーチングポイントP10 へ直線補間移動）
      rb.asyncm(sw=1) # set prefetching ON. （プログラム先読み動作ON　（rb.asyncm(1) でも可能））
      rb.line( P20 ) # line motion to P20. （ティーチングポイントP20 へ直線補間動作で移動する）
      rb.line( P21 ) # line motion to P21. （ティーチングポイントP21へ直線補間動作で移動する）
      rb.join() # end prefetching motion. （先読みしたロボットプログラムの完了を待機する）
      rb.asyncm(sw=2) # set prefetching OFF （プログラム先読み動作OFF　（rb.asyncm(2) でも可能））
      ...
      rb.close()

    """
    res = self.__rblib.joinm()
    if not self.__sysstatus._teachMode:
      liblog("%d = i611Robot.join" % res[1])
    self._internal_hook() ###[HOOK]###
    if res[0] == False:
      return self._error(res[1], res[2])

  #[Public] ############################
  def home(self):
    u"""Shifts all the axes to the 0 degree position. （すべての軸をJoint 座標の0deg に移動する）

    Args:
      None

    Return:
      bool: Returns True if it succeed, otherwise an exception is raised. （成功したときにTrueが返り、それ以外は例外が発生します。）

    **Example**::

      rb.home()

    """
    liblog("i611Robot.home")
    return self.move(Joint(0,0,0,0,0,0))

  #[Public] ############################
  def move(self, *cmd):
    u"""Instructs PTP motion （PTP 動作をする）

    Args:
      cmd(Position or Joint or MotionParam): List of type Position, Joint, or MotionParam （目標座標(PositionまたはJoint)または動作パラメータ（MotionParam）またはそれらのリスト）

    Return:
      bool: Returns True if it succeed, otherwise an exception is raised. （成功したときにTrueが返り、それ以外は例外が発生します。）

    **Example**::

      # Ex1
      #　PTP motion to p10 （位置座標p10 に向かってPTP 動作をする）
      # Condition need to be set by Motion Param in advance （動作条件は、MotionParam で与えられた条件に従う）
      rb.move( p10 )

      # Ex2
      #　PTP motion to p10, followed by p20. （位置座標p10 に向かい、その後p20 に向かってPTP 動作をする）
      # Condition need to be set by Motion Param in advance （動作条件は、MotionParam で与えられた条件に従う）
      rb.move( p10, p20 )

      # Ex3
      # update the condition and PTP motion to p10, followed by p20
      #　（動作条件をMotionParam で変更し、位置座標p10 に向かい、
      #　その後p20 に向かってPTP 動作をする）
      mt=m.MotionParam( posture=1, passm=1, overlap=4.8, zone=20, pose_speed=5.0 )
      rb.move( mt, p10, p20 )

      # Ex4
      # Use crossover counter （クロスオーバーカウンタ情報を使う）
      rb.use_mt(True)
      …
      rb.move( p10 )
      …
      rb.close()

    """
    if not self.__sysstatus._f_no_pause_during_moving:
      self._internal_hook() ###[HOOK]###
    self.__mp = self.__mpdef[:]
    motion_id = []
    for m in cmd:
      if isinstance(m, list):
        for c in m:
          if isinstance(c, Position):
            res = self.__ptpmove(c)
            if res[0] == False:
              return res
            else:
              motion_id.append(res[1])
          elif isinstance(c, Joint):
            res = self.__jntmove(c)
            if res[0] == False:
              return res
            else:
              motion_id.append(res[1])
          elif isinstance(c, MotionParam):
            res = self.__chMotion(c)
            if res[0] == False:
              return res
            else:
              motion_id.append(res[1])
          else:
            return self._error(4, 1)
      else:
        if isinstance(m, Position):
          res = self.__ptpmove(m)
          if res[0] == False:
            return res
          else:
            motion_id.append(res[1])
        elif isinstance(m, Joint):
          res = self.__jntmove(m)
          if res[0] == False:
            return res
          else:
            motion_id.append(res[1])
        elif isinstance(m, MotionParam):
          res = self.__chMotion(m)
          if res[0] == False:
            return res
          else:
            motion_id.append(res[1])
        else:
          return self._error(4, 1)
    return [True] + motion_id

  #[Public] ############################
  def line(self, *cmd):
    u"""Instructs Linear motion (based on linear interpolation) （直線補間動作をする）

    Args:
      cmd(Position or Joint or MotionParam): List of type Position, Joint, or MotionParam （目標座標(PositionまたはJoint)または動作パラメータ（MotionParam）またはそれらのリスト）

    Return:
      bool: Returns True if it succeed, otherwise an exception is raised. （成功したときにTrueが返り、それ以外は例外が発生します。）

    **Example**::

      # Ex1
      # line motion to p10. （座標位置p10 に向かって直線補間動作をする）
      # Condition need to be set by Motion Param in advance. （動作条件は、MotionParam で与えられた条件に従う）
      rb.line(p10)

      # Ex2
      # line motion to p10, followed by p20.
      # （座標位置p10 に向かい、その後p20 に向かって直線補間動作をする）
      # Condition need to be set by Motion Param in advance （動作条件は、MotionParam で与えられた条件に従う）
      rb.line( p10, p20 )

      # Ex3
      # update the condition and line motion to p10, followed by p20
      # （動作条件をMotionParam で変更し、座標位置p10 に向かい、
      # その後p20 に向かって直線補間動作をする）
      mt=m.MotionParam( posture=1, passm=1, overlap=4.8, zone=20, pose_speed=5.0 )
      rb.line( mt, p10, p20 )

    """
    if not self.__sysstatus._f_no_pause_during_moving:
      self._internal_hook() ###[HOOK]###
    self.__mp = self.__mpdef[:]
    motion_id = []
    for m in cmd:
      if isinstance(m, list):
        for c in m:
          if isinstance(c, Position) or isinstance(c, Joint):
            res = self.__cpmove(c)
            if res[0] == False:
              return res
            else:
              motion_id.append(res[1])
          elif isinstance(c, MotionParam):
            res = self.__chMotion(c)
            if res[0] == False:
              return res
            else:
              motion_id.append(res[1])
          else:
            return self._error(3, 1)
      else:
        if isinstance(m, Position) or isinstance(m, Joint):
          res = self.__cpmove(m)
          if res[0] == False:
              return res
          else:
            motion_id.append(res[1])
        elif isinstance(m, MotionParam):
          res = self.__chMotion(m)
          if res[0] == False:
              return res
          else:
            motion_id.append(res[1])
        else:
          return self._error(3, 1)
    return [True] + motion_id

  #[Public] ############################
  def optline(self, *cmd):
    u"""Instructs optimized Linear motion （最適直線補間動作をする）

    Args:
      cmd(Position or Joint or MotionParam): List of type Position, Joint, or MotionParam （目標座標(PositionまたはJoint)または動作パラメータ（MotionParam）またはそれらのリスト）

    Return:
      bool: Returns True if it succeed, otherwise an exception is raised. （成功したときにTrueが返り、それ以外は例外が発生します。）

    Note:
      * optline speed is relating to jnt_speed (not lin_speed) in motion parameters.
      * （optline( ) の速度は、lin_speed（直線補間動作）ではなく jnt_speed（PTP 動作・Joint 動作・最適直線補間動作）で設定します。）

    **Example**::

      # Ex1
      #　optline motion to p10 （位置座標p10 に向かって最適直線補間動作をする）
      # Condition need to be set by Motion Param in advance. （動作条件は、MotionParam で与えられた条件に従う）
      rb.optline( p10 )

      # Ex2
      # optline motion to p10, followed by p20.
      #　（位置座標p10 に向かい、その後p20 に向かって最適直線補間動作をする）
      # Condition need to be set by Motion Param in advance. （動作条件は、MotionParam で与えられた条件に従う）
      rb.optline( p10, p20 )

      # Ex3
      # update the condition and line motion to p10, followed by p20
      #（動作条件をMotionParam で変更し、位置座標p10 に向かい、
      # その後p20 に向かって最適直線補間動作をする）
      mt=m.MotionParam( posture=1, 座標passm=1, overlap=4.8, zone=20, pose_speed=5.0 )
      rb.optline( mt, p10, p20 )

    """
    if not self.__sysstatus._f_no_pause_during_moving:
      self._internal_hook() ###[HOOK]###
    self.__mp = self.__mpdef[:]
    motion_id = []
    for m in cmd:
      if isinstance(m, list):
        for c in m:
          if isinstance(c, Position) or isinstance(c, Joint):
            res = self.__optcpmove(c)
            if res[0] == False:
              return res
            else:
              motion_id.append(res[1])
          elif isinstance(c, MotionParam):
            res = self.__chMotion(c)
            if res[0] == False:
              return res
            else:
              motion_id.append(res[1])
          else:
            return self._error(4, 1)
      else:
        if isinstance(m, Position) or isinstance(m, Joint):
          res = self.__optcpmove(m)
          if res[0] == False:
              return res
          else:
            motion_id.append(res[1])
        elif isinstance(m, MotionParam):
          res = self.__chMotion(m)
          if res[0] == False:
              return res
          else:
            motion_id.append(res[1])
        else:
          return self._error(4, 1)
    return [True] + motion_id


  #[Public] ############################
  #pram [dx,dy,dz,drz,dry,drx]
  def toolmove(self, *arg1, **arg2):
    u"""Instructs a motion relative to the tool coordinate system. The robot will move according to the motion parameter set by the MotionParam method. （ツール座標系で相対動作をする）

    Args:
      dx(float): x-offset[mm] in the tool coordinate system. （ツール座標系でのX軸方向の移動量[mm]）
      dy(float): y-offset[mm] in the tool coordinate system. （ツール座標系でのY軸方向の移動量[mm]）
      dz(float): z-offset[mm] in the tool coordinate system. （ツール座標系でのZ軸方向の移動量[mm]）
      drz(float): rz-offset[deg] in the tool coordinate system. （ツール座標系でのZ軸まわりの移動量[deg]）
      dry(float): ry-offset[deg] in the tool coordinate system. （ツール座標系でのY軸まわりの移動量[deg]）
      drx(float): rx-offset[deg] in the tool coordinate system. （ツール座標系でのX軸まわりの移動量[deg]）

    Return:
      bool: Returns True if it succeed, otherwise an exception is raised. （成功したときにTrueが返り、それ以外は例外が発生します。）

    **Example**::

      # Define Position data [dx, dy, dz, drz, dry, drx]
      # （Positon 型[dx, dy, dz, drz, dry, drx] のティーチングデータをリストで定義しておく。）
      p10=Position( 95, -280, 240, 154, 80, -114 )

      # tool move from p10 to offset (dx=15) position.
      # （座標位置p10 へ向かった後、ツール座標系でdx=15mm の相対動作をする）
      ...
      rb.move( p10 )
      rb.toolmove( dx=15 )
      ...
      rb.close()

    """
    p = i611_common._args([0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              ['dx', 'dy', 'dz', 'drz', 'dry', 'drx'],
              [float, float, float, float, float, float],
              *arg1, **arg2)
    if p[0] == False:
      return self._error(4, p[1],"toolmove")
    else:
      _speed = self.__mp2[0]
      _acct = self.__mp2[2]
      _dacct = self.__mp2[3]
      if not self.__sysstatus._f_no_pause_during_moving:
        self._internal_hook() ###[HOOK]###
      res = self.__rblib.trmove(p[1], p[2], p[3], p[4], p[5], p[6],_speed,_acct,_dacct)
      if not self.__sysstatus._teachMode:
        liblog(("%d = i611Robot.toolmove" % res[1]),p[1], p[2], p[3], p[4], p[5], p[6],_speed,_acct,_dacct)
      self._internal_hook() ###[HOOK]###
      if res[0] == False:
        return self._error(res[1], res[2])
      else:
        return res

  #[Public] ############################
  def motionparam(self, *arg1, **arg2):
    u"""Sets motion condition with data of type MotionParam. （動作パラメータを設定する）

    Args:
      MotionParam: parameters or instance of MotionParam （動作パラメータ）

    Return:
      bool: Returns True if it succeed, otherwise an exception is raised. （成功したときにTrueが返り、それ以外は例外が発生します。）

    **Example**::

      # Ex1: set motion parameter by MotionParam instance
      # 例1：MotionParam 型のインスタンスで設定する
      m=MotionParam()
      rb.motionparam( m )

      # Ex2: set motion parameter by specifing each parameter.
      # 例2：MotionParam 型のメンバ変数のキーワードで設定する
      rb.motionparam( posture=1, passm=1, overlap=4.8, zone=20, pose_speed=5.0 )

    """
    if len(arg1) == 1 and isinstance(arg1[0], MotionParam):
      self.__mpdef = arg1[0].mp2list()
      res = self.__chMotion(arg1[0])
    else:
      m = MotionParam(*arg1, **arg2)
      self.__mpdef = m.mp2list()
      res = self.__chMotion(m)
    return res

  #[Public] ############################
  def getmotionparam(self):
    u"""Obtains the motion condition currently being set. （現在の動作パラメータを取得する）

    Args:
      None

    Return:
      MotionParam: instance of MotionParam （動作パラメータ）

    **Example**::

      # refer each parameter through MotionParam instance.
      #MotionParam のインスタンスを参照
      t_lin_speed=rb.getmotionparam().lin_speed
      t_lin_overlap=rb.getmotionparam().overlap

    """
    return MotionParam(self.__mp)

  #[Public] ############################
  def override(self, ovr):
    u"""Performs override. （オーバーライドを行う）

    Args:
      ovr(int): Sets the motion speed rate of robot (%) （motionparam() で設定したロボットの駆動速度に倍率[%]をかけて速度を調整します。（省略不可））

        * Range （設定範囲）: 0 ～ 200

    Return:
      bool: Returns True if it succeed, otherwise an exception is raised. （成功したときにTrueが返り、それ以外は例外が発生します。）

    **Example**::

      # set override to 50%
      # （オーバーライドを50％に設定する）
      rb.override( 50 )

    """
    liblog("i611Robot.override",ovr)
    res = i611_common._chkparam(ovr, p_type = [int, float], min = 0, max = 200)
    if res[0] == False:
      return self._error(4, res[1])
    # __mp2 = [lin_speed, jnt_speed, acctime, dacctime, posture, passm, overlap, zone, pose_speed, ik_solver_option]
    self.__ovr = float(ovr) / 100.
    self.__mp2 = [x * self.__ovr for x in self.__mp[:2]] + \
      self.__mp[2:8] + [self.__mp[8] * self.__ovr] + [self.__mp[9]]

  #[Internal] --------------------------
  def set_restrict(self, *area):
    u"""Private (非公開)"""
    liblog("i611Robot.set_restrict")
    temp = []
    if area == []:
      self.__restrict = []
    for m in area:
      if isinstance(m, list):
        if m == []:
          self.__restrict = []
          return [True]
        for c in m:
          if isinstance(c, Area_box):
            temp.append(c)
          elif isinstance(c, Area_cube):
            temp.append(c)
          elif isinstance(c, Area_cyl):
            temp.append(c)
          else:
            return self._error(4, 1)
      else:
        if isinstance(m, Area_box):
          temp.append(c)
        elif isinstance(m, Area_cube):
          temp.append(c)
        elif isinstance(m, Area_cyl):
          temp.append(c)
        else:
          return self._error(4, 1)
    self.__restrict = temp[:]
    return [True]

  #[Internal] --------------------------
  def areacheck(self, pos):
    u"""Private (非公開)"""
    liblog("i611Robot.areacheck")
    if self.__restrict == []:
      return False
    if isinstance(pos, Position):
      p = pos
    elif isinstance(pos, Joint):
      p = self.Joint2Position(pos)
    else:
      return self._error(4, 1)
    res = False
    for m in self.__restrict:
      res = res or m.inarea(p)
    return res

  #[Public] ############################
  def settool(self, *arg1, **arg2):
    u"""Sets up tool offsets. （ツールオフセットを設定する）

    Args:
      id(int): tool No.（ツール番号）
      offx(float): x-offset[mm] in the tool coordinate system. （ツール座標系でのX軸のツールオフセット量[mm]）
      offy(float): y-offset[mm] in the tool coordinate system. （ツール座標系でのY軸のツールオフセット量[mm]）
      offz(float): z-offset[mm] in the tool coordinate system. （ツール座標系でのZ軸のツールオフセット量[mm]）
      offrz(float): rz-offset[deg] in the tool coordinate system. （ツール座標系でのZ軸まわりのツールオフセット量[deg]）
      offry(float): ry-offset[deg] in the tool coordinate system. （ツール座標系でのY軸まわりのツールオフセット量[deg]）
      offrx(float): rx-offset[deg] in the tool coordinate system. （ツール座標系でのX軸まわりのツールオフセット量[deg]）

    Return:
      bool: Returns True if it succeed, otherwise an exception is raised. （成功したときにTrueが返り、それ以外は例外が発生します。）

    **Example**::

      # 1. Set tooloffset to tool No.1.（ツールオフセット（ツールNo.1）を設定する）
      rb.settool( 1, 0.0, 0.0, 200.0, 0.0, 0.0, 0.0 )

      # 2. Select tool No.1. （ツールオフセットNo.1 を選択する）
      rb.changetool( 1 )


    """
    p = i611_common._args([0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              ['id', 'offx', 'offy', 'offz', 'offrz', 'offry', 'offrx'],
              [int, float, float, float, float, float, float],
              *arg1, **arg2)
    if p[0] == False:
      return self._error(4, p[1], "settool")
    else:
      if not self.__sysstatus._teachMode:
        liblog("i611Robot.settool",p[1], p[2], p[3], p[4], p[5], p[6], p[7])
      res = self.__rblib.settool(p[1], p[2], p[3], p[4], p[5], p[6], p[7])
      if res[0] == False:
        return self._error(res[1], res[2])
      else:
        return res

  #[Public] ############################
  def changetool(self, tid):
    u"""Selects a tool offset. （ツールオフセットを選択する）

    Args:
      tid: tool No. （ツール番号）

        * 0: deselect tool（ツールオフセットを解除する）
        * 1 - 8: select tool No. （ツールオフセットを選択する）

    Return:
      bool: Returns True if it succeed, otherwise an exception is raised. （成功したときにTrueが返り、それ以外は例外が発生します。）

    **Example**::

      # 1. Set tooloffset to tool No.1.（ツールオフセット（ツールNo.1）を設定する）
      rb.settool( 1, 0.0, 0.0, 200.0, 0.0, 0.0, 0.0 )

      # 2. Select tool No.1. （ツールオフセットNo.1 を選択する）
      rb.changetool( 1 )


    """
    self._internal_hook() ###[HOOK]###
    if not self.__sysstatus._teachMode:
      liblog("i611Robot.changetool",tid)
    res = i611_common._chkparam(tid, p_type = int, min = 0, max = 8)
    if res[0] == False:
      return self._error(4, res[1])
    else:
      res = self.__rblib.changetool(tid)
      if res[0] == False:
        return self._error(res[1], res[2])
      else:
        return res

  #[Public] ############################
  def set_mdo(self, mdoid, portno, value, kind, distance):
    u"""Sets the MDO motion. （MDO 動作の設定をする）

    Args:
      mdoid(int): MDO control number （MDO管理番号）

        * Range （設定範囲）: 1 - 8

      portno(int): Port Output number （ポート出力番号）

        * Range （設定範囲）: 0 - 12287

      value(int): Output value （I/O 出力）

        * 0: Short （短絡）
        * 1: Open （開放）

      kind(int): Condition （条件）

        * 1: the specified amount of distance away from the start point （始点から一定距離を離れた）
        * 2: within the specified amount of distance from the end point （終点から一定距離内に入った）

      distance(float): distance[mm] （距離[mm]）

        * Range （設定範囲）: 0.0 ～

    Return:
      bool: Returns True if it succeed, otherwise an exception is raised. （成功したときにTrueが返り、それ以外は例外が発生します。）

    **Example**::

      # Ex1: Set parameters
      rb.set_mdo( 1, 23, 0, 1, 30 ) # set condition in MDO 1. （MDO 管理番号１に設定）
      rb.set_mdo( 8, 23, 1, 2, 10 ) # set condition in MDO 8. （MDO 管理番号８に設定）

      # Ex2: Set parameters by keyword
      # 例2：キーワード
      rb.set_mdo( mdoid=1, portno=23, value=0, kind=1, distance=30 )
      rb.set_mdo( mdoid=8, portno=23, value=1, kind=2, distance=10 )

    """
    liblog("i611Robot.set_mdo", mdoid, portno, value, kind, distance )
    res = i611_common._chkparam(mdoid,  p_type = int, min = 1, max = 8)
    if res[0] == False:
      return self._error(4, res[1])
    res = i611_common._chkparam(portno, p_type = int, min = 0)
    if res[0] == False:
      return self._error(4, res[1])
    res = i611_common._chkparam(value,  p_type = int, min = 0, max = 1)
    if res[0] == False:
      return self._error(4, res[1])
    res = i611_common._chkparam(kind,   p_type = int, min = 1, max = 2)
    if res[0] == False:
      return self._error(4, res[1])
    res = i611_common._chkparam(distance, p_type = [int, float], min = 0.0)
    if res[0] == False:
      return self._error(4, res[1])
    res = self.__rblib.set_mdo(mdoid, portno, value, kind, distance)
    if res[0] == False:
      return self._error(res[1], res[2])
    else:
      return res

  #[Public] ############################
  def enable_mdo(self, bitfield):
    u"""Enables MDO motion. （MDO 動作を有効にする）

    Args:
      bitfield(int): MDO control number （管理番号（ビットフィールド））

        * This argument sets a bit(s) corresponding to control number(s) to be enabled. （有効にするMDO の管理番号に該当するbit を立てる）
        * Range （設定範囲）: 0 - 255

    Return:
      bool: Returns True if it succeed, otherwise an exception is raised. （成功したときにTrueが返り、それ以外は例外が発生します。）

    **Example**::

      # set condition （MDO 動作の設定をしておきます(*3)）
      rb.set_mdo( 1, 23, 0, 1, 30 )
      rb.set_mdo( 8, 23, 1, 2, 10 )

      # enable MDO （MDO 動作を有効にする）
      # Ex1: Set parameters
      # 例1：数値指定
      rb.enable_mdo( 129 )　　　　　# Enable MDO 1,8 （MDO 管理番号１、８を有効にする）

      # Ex2: Set parameters by keyword
      # 例2：キーワード
      rb.enable_mdo( bitfield=129 )

    """
    liblog("i611Robot.enable_mdo", bitfield )
    res = i611_common._chkparam(bitfield,  p_type = int, min = 0, max = 255)
    if res[0] == False:
      return self._error(4, res[1])
    res = self.__rblib.enable_mdo(bitfield)
    if res[0] == False:
      return self._error(res[1], res[2])
    else:
      return res

  #[Public] ############################
  def disable_mdo(self, bitfield):
    u"""Disables the MDO motion （MDO 動作を無効にする）

    Args:
      bitfield(int):MDO control number （管理番号（ビットフィールド））

        * This argument sets a bit(s) corresponding to control number(s) to　be disabled. （無効にするMDO の管理番号に該当するbit を立てる）
        * Range （設定範囲）: 0 - 255

    Return:
      bool: Returns True if it succeed, otherwise an exception is raised. （成功したときにTrueが返り、それ以外は例外が発生します。）

    **Example**::

      # set condition （MDO 動作の設定をしておきます(*2)）
      rb.set_mdo( 1, 23, 0, 1, 30 )
      rb.set_mdo( 8, 23, 1, 2, 10 )
      rb.enable_mdo(129)　　　　　　 # Enable MDO （MDO 管理番号１、８を有効）

      # disable MDO （MDO 動作を無効にする）
      # Ex1: Set parameters
      # 例1：数値指定
      rb.disable_mdo( 129 )　　　　　 # Disable MDO （MDO 管理番号１～８を無効）

      # Ex2: Set parameters by keyword
      # 例2：キーワード
      rb.disable_mdo( bitfield=129 )

    """
    liblog("i611Robot.disable_mdo", bitfield )
    res = i611_common._chkparam(bitfield,  p_type = int, min = 0, max = 255)
    if res[0] == False:
      return self._error(4, res[1])
    res = self.__rblib.disable_mdo(bitfield)
    if res[0] == False:
      return self._error(res[1], res[2])
    else:
      return res

  #[Public] ############################
  @staticmethod
  def use_mt(flg):
    u"""set if crossover counter is used or not. （クロスオーバーカウンタの有効/ 無効を設定する）

    Args:
      flg(bool): True if crossover counter is used. （使用する場合はTrue）

    Return:
      None

    **Example**::

      rb.use_mt(True)

    """
    global _use_mt
    _use_mt = bool(flg)

  #[Public] ############################
  def getpos(self):
    u"""Obtains the current manipulator positional data of type Position. （マニピュレータの現在位置をPosition 型で取得する）

    Args:
      None

    Return:
      Position: Position instance of current position （現在位置(Position型））

    **Example**::

      rb.home()
      pos01=rb.getpos()

    """
    # Positionデータには、常に多回転データを入れておく。
    res = self.__rblib.mark_mt()
    if res[0] == False:
      return self._error(res[1], res[2], "getpos")
    else:
      if not self.__sysstatus._teachMode and self.last_pos != res[1:]:
        #liblog("i611Robot.getpos",res[1:])  # no output
        self.last_pos = res[1:]
      return Position(res[1:7], _BASE, res[7], res[9])

  #[Public] ############################
  def getjnt(self):
    u"""Obtains the current manipulator positional data as of type Joint. （マニピュレータの現在位置をJoint 型で取得する）

    Args:
      None

    Return:
      Joint: Joint instance of current position （現在位置(Joint型））

    **Example**::

      rb.home()
      jnt01=rb.getjnt()

    """
    res = self.__rblib.jmark()
    if res[0] == False:
      return self._error(res[1], res[2],"getjnt")
    else:
      if not self.__sysstatus._teachMode and self.last_jnt != res[1:]:
        #liblog("i611Robot.getjnt",res[1:])  # no output
        self.last_jnt = res[1:]
      return Joint(res[1:7])

  #[Public] ############################
  def Joint2Position(self, *jnt):
    u"""Converts coordinates from the Joint coordinate system to the Position coordinate system　（Joint 座標値からPosition 座標値へ変換する）

    Args:
      jnt(Joint or Joint list): Joint instance to convert （変換の対象となる Joint 座標）

    Return:
      Position: Converted Position instance （Position座標に変換した座標値）

    **Example**::

      j10=Joint( 0, 30, 60, 0, 90, 90 )

      #Convert from Joint to Position （Position 座標値へ変換（ j10 →変換→ p10 )）
      p10=rb.Joint2Position( j10 )

    """
    v = []
    for m in jnt:
      if isinstance(m, list):
        for c in m:
          if isinstance(c, Joint):
            a = c.jnt2list()
            b = self.__rblib.j2r_mt(a[0],a[1],a[2],a[3],a[4],a[5],1)
            if b[0] == True:
              v.append(Position(b[1:7], _BASE, b[7], b[9]))
            else:
              v.append(0)
          else:
            return self._error(4, 1, "Joint2Position")
      else:
        if isinstance(m, Joint):
          a = m.jnt2list()
          b = self.__rblib.j2r_mt(a[0],a[1],a[2],a[3],a[4],a[5],1)
          if b[0] == True:
            v.append(Position(b[1:7], _BASE, b[7], b[9]))
          else:
            v.append(0)
        else:
          return self._error(4, 1, "Joint2Position")
    if len(v) > 1:
      return v
    else:
      return v[0]

  #[Public] ############################
  def Position2Joint(self, *pos):
    u"""Converts coordinates from the Position coordinate system to the Joint coordinate system　（Position 座標値からJoint 座標値へ変換する）

    Args:
      pos(Position or Position list): Position instance to convert （変換の対象となるPosition 座標）

    Return:
      Joint: Converted Joint instance （Joint座標に変換した座標値）

    **Example**::

      p10=Position( -50, -250, 350, 90, 0, 180 )

      #Convert from Joint to Position （Joint 型座標値へ変換( p10 →変換→ j10 )）
      j10=rb.Position2Joint( p10 )

    """
    _ik_solver_option = self.__mp[9]
    v = []
    for m in pos:
      if isinstance(m, list):
        for c in m:
          if isinstance(c, Position):
            a = c.position(True)
            b = self.__rblib.r2j_mt(a[0],a[1],a[2],a[3],a[4],a[5],a[7],1,a[8],_ik_solver_option)
            if b[0] == True:
              v.append(Joint(b[1:7]))
            else:
              v.append(0)
          else:
            return self._error(4, 1, "Position2Joint")
      else:
        if isinstance(m, Position):
            a = m.position(True)
            b = self.__rblib.r2j_mt(a[0],a[1],a[2],a[3],a[4],a[5],a[7],1,a[8],_ik_solver_option)
            if b[0] == True:
              v.append(Joint(b[1:7]))
            else:
              v.append(0)
        else:
          return self._error(4, 1, "Position2Joint")
    if len(v) > 1:
      return v
    else:
      return v[0]

  #[Internal] --------------------------
  def _error(self, div, code, fnc=""):
    u"""Private (非公開)"""
#    res = _Err_mcs.mcserr(src2, div, code)
    if fnc != "":
      fnc = "i611Robot."+fnc
    res = liberr_sub(div,code,fnc,no_logout=self.__sysstatus._teachMode)

    #No permission
    if (div, code) == (3,1):
      if self.__sysstatus._enbErr == False:
        return [False, 3, 1]
      else:
        self.__cause_fatal_exception(11)

    #非常停止(EMO)
    if (div, code) == (3,15):
      if not self.__sysstatus._ev_emo.is_set():
        self.__sysstatus._ev_emo.set()
      return [False, 3, 15]

    #減速停止(STOP)
    if (div, code) == (3,16):
      if not self.__sysstatus._ev_stop.is_set():
        self.__sysstatus._ev_stop.set()
      return [False, 3, 16]

    if res[0]:
      return [False, div, code]
    else:
      raise Robot_error(res[3])

  #[Internal] --------------------------
  def __ptpmove(self, pram):
    u"""Private (非公開)"""
    #[x, y, z, rz, ry, rx, parent, posture]
    _x, _y, _z, _rz, _ry, _rx = pram.position()[0:6]
    if pram.pos[7] == -1:
      _posture = self.__mp2[4]
    else:
      _posture = pram.pos[7]
    _multiturn = pram.pos[8]
    _rbcoord = 1
    _speed = self.__mp2[1]
    _acct = self.__mp2[2]
    _dacct = self.__mp2[3]
    _ik_solver_option = self.__mp2[9]

    if self.accel_limit:
      _speed = self.convert_accel_ptp(_x, _y, _z, _rz, _ry, _rx, _posture,
        _rbcoord, _multiturn, _ik_solver_option, _speed, _acct, _dacct)

    self._moveing = True
    if _use_mt:
      res = self._ptpmove_mt(_x, _y, _z, _rz, _ry, _rx, _posture, _rbcoord, _multiturn, _ik_solver_option, _speed, _acct, _dacct)
      if not self.__sysstatus._teachMode:
        liblog(("%d = i611Robot.ptpmove_mt" % res[1]),_x, _y, _z, _rz, _ry, _rx, _posture, _rbcoord, _multiturn, _ik_solver_option, _speed, _acct, _dacct)
    else:
      res = self._ptpmove(_x, _y, _z, _rz, _ry, _rx, _posture, _rbcoord, _speed, _acct, _dacct)
      if not self.__sysstatus._teachMode:
        liblog(("%d = i611Robot.ptpmove" % res[1]),_x, _y, _z, _rz, _ry, _rx, _posture, _rbcoord, _speed, _acct, _dacct)
    self._moveing = False
    if res[0] == False:
      retval = self._error(res[1], res[2])
      self._internal_hook() ###[HOOK]###
      return retval
    else:
      self._internal_hook() ###[HOOK]###
      return res

  #[Internal] --------------------------
  def __cpmove(self, pram):
    u"""Private (非公開)"""
    #[x, y, z, rz, ry, rx, parent, posture]
    if isinstance(pram, Position):
      _x, _y, _z, _rz, _ry, _rx = pram.position()[0:6]
      if pram.pos[7] == -1:
        _posture = self.__mp2[4]
      else:
        _posture = pram.pos[7]
    elif isinstance(pram, Joint):
      #pylint: disable=maybe-no-member
      p = self.Joint2Position(pram).pos
      _x, _y, _z, _rz, _ry, _rx = p[0:6]
      _posture = p[7]
    else:
      return self._error(4, 1, "cpmove")
    _rbcoord = 1
    _speed = self.__mp2[0]
    _acct = self.__mp2[2]
    _dacct = self.__mp2[3]

    if self.accel_limit:
      _speed = self.convert_accel_cp(_x, _y, _z, _rz, _ry, _rx, _posture,
        _rbcoord, _speed, _acct, _dacct)

    self._moveing = True
    res = self._cpmove(_x, _y, _z, _rz, _ry, _rx, _posture, _rbcoord, _speed, _acct, _dacct)
    self._moveing = False
    if not self.__sysstatus._teachMode:
      liblog(("%d = i611Robot.cpmove" % res[1]), _x, _y, _z, _rz, _ry, _rx, _posture, _rbcoord, _speed, _acct, _dacct)
    if res[0] == False:
      retval = self._error(res[1], res[2])
      self._internal_hook() ###[HOOK]###
      return retval
    else:
      self._internal_hook() ###[HOOK]###
      return res

  #[Internal] --------------------------
  def __optcpmove(self, pram):
    u"""Private (非公開)"""
    #[x, y, z, rz, ry, rx, parent, posture]
    if isinstance(pram, Position):
      _x, _y, _z, _rz, _ry, _rx = pram.position()[0:6]
      if pram.pos[7] == -1:
        _posture = self.__mp2[4]
      else:
        _posture = pram.pos[7]
    elif isinstance(pram, Joint):
      #pylint: disable=maybe-no-member
      p = self.Joint2Position(pram).pos
      _x, _y, _z, _rz, _ry, _rx = p[0:6]
      _posture = p[7]
    else:
      return self._error(4, 1, "optcpmove")
    _rbcoord = 1
    _speed = self.__mp2[1]
    _acct = self.__mp2[2]
    _dacct = self.__mp2[3]

    if self.accel_limit:
      _speed = self.convert_accel_optcp(_x, _y, _z, _rz, _ry, _rx, _posture,
        _rbcoord, _speed, _acct, _dacct)
    self._moveing = True
    res = self._optcpmove(_x, _y, _z, _rz, _ry, _rx, _posture, _rbcoord, _speed, _acct, _dacct)
    self._moveing = False
    if not self.__sysstatus._teachMode:
      liblog(("%d = i611Robot.optcpmove" % res[1]), _x, _y, _z, _rz, _ry, _rx, _posture, _rbcoord, _speed, _acct, _dacct)
    if res[0] == False:
      retval = self._error(res[1], res[2])
      self._internal_hook() ###[HOOK]###
      return retval
    else:
      self._internal_hook() ###[HOOK]###
      return res

  #[Internal] --------------------------
  def __jntmove(self, pram):
    u"""Private (非公開)"""
    #[ax1, ax2, ax3, ax4, ax5, ax6]
    _ax1, _ax2, _ax3, _ax4, _ax5, _ax6 = pram.jnt2list()[0:6]
    _speed = self.__mp2[1]
    _acct = self.__mp2[2]
    _dacct = self.__mp2[3]

    self._moveing = True
    res = self._jntmove(_ax1, _ax2, _ax3, _ax4, _ax5, _ax6, _speed, _acct, _dacct)
    self._moveing = False
    if not self.__sysstatus._teachMode:
      liblog(("%d = i611Robot.jntmove" % res[1]),  _ax1, _ax2, _ax3, _ax4, _ax5, _ax6, _speed, _acct, _dacct)
    if res[0] == False:
      retval = self._error(res[1], res[2])
      self._internal_hook() ###[HOOK]###
      return retval
    else:
      self._internal_hook() ###[HOOK]###
      return res

  #[Internal] --------------------------
  def __chMotion(self, pram):
    u"""Private (非公開)"""
    if not isinstance(pram, MotionParam):
      return self._error(4, 1, "__chMotion")
    else:
      # Check parameter update
      ##Xself.__mp = pram.mp2list()
      new_mp = pram.mp2list()

      update_overlap = False
      update_passm   = False
      update_zone    = False
      update_slspeed = False

      if new_mp[6] != self.__mp[6]: # Check overlap update
        update_overlap = True
      if new_mp[5] != self.__mp[5]: # Check passm update
        update_passm = True
      if new_mp[7] != self.__mp[7]: # Check zone update
        update_zone = True
      if new_mp[8] != self.__mp[8]: # Check slspeed update
        update_slspeed = True

      if self.first_parameter_update:
        self.first_parameter_update = False
        update_overlap = True
        update_passm   = True
        update_zone    = True
        update_slspeed = True

      self.__mp = new_mp

      if self.__mp[8] < 1.0:
        self.__mp[8] = 1.0

      # __mp2 = [lin_speed, jnt_speed, acctime, dacctime, posture, passm, overlap, zone, pose_speed, ik_solver_option]
      self.__mp2 = [x * self.__ovr for x in self.__mp[:2]] + \
        self.__mp[2:8] + [self.__mp[8] * self.__ovr] + [self.__mp[9]]
      if not self.__sysstatus._teachMode:
        liblog("i611Robot.chMotion",self.__mp[0], self.__mp[1], self.__mp[2], self.__mp[3],
          self.__mp[4], self.__mp[5], self.__mp[6], self.__mp[7], self.__mp[8], self.__mp[9])

      if update_overlap:
        res = self.__rblib.overlap(self.__mp[6])
        if res[0] == False:
          return self._error(res[1], res[2])

      if update_passm:
        res = self.__rblib.passm(self.__mp[5])
        if res[0] == False:
          return self._error(res[1], res[2])

      if update_zone:
        res = self.__rblib.zone(self.__mp[7])
        if res[0] == False:
          return self._error(res[1], res[2])

      if update_slspeed:
        res = self.__rblib.slspeed(self.__mp[8])
        if res[0] == False:
          return self._error(res[1], res[2])

      return [True]

  #[Internal] --------------------------
  def _open(self):
    u"""Private (非公開)"""
    return self.__rblib.open()

  #[Internal] --------------------------
  def _close(self):
    u"""Private (非公開)"""
    return self.__rblib.close()

  #[Internal] --------------------------
  def _abortm(self):
    u"""Private (非公開)"""
    return self.__rblib.abortm()

  #[Internal] --------------------------
  def _svctrl(self, sw):
    u"""Private (非公開)"""
    return self.__rblib.svctrl(sw)

  #[Internal] --------------------------
  def _plsmove(self, ax1, ax2, ax3, ax4, ax5, ax6, speed, acct, dacct):
    u"""Private (非公開)"""
    return self.__rblib.plsmove(ax1, ax2, ax3, ax4, ax5, ax6, speed, acct, dacct)

  #[Internal] --------------------------
  def _mtrmove(self, ax1, ax2, ax3, ax4, ax5, ax6, speed, acct, dacct):
    u"""Private (非公開)"""
    return self.__rblib.mtrmove(ax1, ax2, ax3, ax4, ax5, ax6, speed, acct, dacct)

  #[Internal] --------------------------
  def _jntmove(self, ax1, ax2, ax3, ax4, ax5, ax6, speed, acct, dacct):
    u"""Private (非公開)"""
    return self.__rblib.jntmove(ax1, ax2, ax3, ax4, ax5, ax6, speed, acct, dacct)

  #[Internal] --------------------------
  def _ptpmove(self, x, y, z, rz, ry, rx, posture, rbcoord, speed, acct, dacct):
    u"""Private (非公開)"""
    return self.__rblib.ptpmove(x, y, z, rz, ry, rx, posture, rbcoord, speed, acct, dacct)

  #[Internal] --------------------------
  def _ptpmove_mt(self, x, y, z, rz, ry, rx, posture, rbcoord, multiturn, ik_solver_option, speed, acct, dacct):
    u"""Private (非公開)"""
    return self.__rblib.ptpmove_mt(x, y, z, rz, ry, rx, posture, rbcoord, multiturn, ik_solver_option, speed, acct, dacct)

  #[Internal] --------------------------
  def _cpmove(self, x, y, z, rz, ry, rx, posture, rbcoord, speed, acct, dacct):
    u"""Private (非公開)"""
    return self.__rblib.cpmove(x, y, z, rz, ry, rx, posture, rbcoord, speed, acct, dacct)

  #[Internal] --------------------------
  def _optcpmove(self, x, y, z, rz, ry, rx, posture, rbcoord, speed, acct, dacct):
    u"""Private (非公開)"""
    return self.__rblib.optcpmove(x, y, z, rz, ry, rx, posture, rbcoord, speed, acct, dacct)

  #[Internal] --------------------------
  def _ptpplan(self, ex, ey, ez, erz, ery, erx, eposture, erbcoord, speed, acct, dacct):
    u"""Private (非公開)"""
    return self.__rblib.ptpplan(ex, ey, ez, erz, ery, erx, eposture, erbcoord, speed, acct, dacct)

  #[Internal] --------------------------
  def _ptpplan_mt(self, ex, ey, ez, erz, ery, erx, eposture, erbcoord, emultiturn, ik_solver_option, speed, acct, dacct):
    u"""Private (非公開)"""
    return self.__rblib.ptpplan_mt(ex, ey, ez, erz, ery, erx, eposture, erbcoord, emultiturn, ik_solver_option, speed, acct, dacct)

  #[Internal] --------------------------
  def _cpplan(self, ex, ey, ez, erz, ery, erx, eposture, erbcoord, speed, acct, dacct):
    u"""Private (非公開)"""
    return self.__rblib.cpplan(ex, ey, ez, erz, ery, erx, eposture, erbcoord, speed, acct, dacct)

  #[Internal] --------------------------
  def _optcpplan(self, ex, ey, ez, erz, ery, erx, eposture, erbcoord, speed, acct, dacct):
    u"""Private (非公開)"""
    return self.__rblib.optcpplan(ex, ey, ez, erz, ery, erx, eposture, erbcoord, speed, acct, dacct)

  #[Internal] --------------------------
  def _cprplan(self, dx, dy, dz, drz, dry, drx, speed, acct, dacct):
    u"""Private (非公開)"""
    return self.__rblib.cprplan(dx, dy, dz, drz, dry, drx, speed, acct, dacct)

  #[Internal] --------------------------
  def _r2j_mt(self, x, y, z, rz, ry, rx, posture, rbcoord, multiturn,ik_solver_option):
    u"""Private (非公開)"""
    return self.__rblib.r2j_mt(x, y, z, rz, ry, rx, posture, rbcoord, multiturn,ik_solver_option)

  #[Internal] --------------------------

  #[Internal] --------------------------
  def _mark(self):
    u"""Private (非公開)"""
    return self.__rblib.mark()

  #[Internal] --------------------------
  def _mark_mt(self):
    u"""Private (非公開)"""
    return self.__rblib.mark_mt()

  #[Internal] --------------------------
  def _jmark(self):
    u"""Private (非公開)"""
    return self.__rblib.jmark()

  #[Internal] --------------------------
  def _pmark(self, sw):
    u"""Private (非公開)"""
    return self.__rblib.pmark(sw)

  #[Internal] --------------------------
  def _joinm(self):
    u"""Private (非公開)"""
    return self.__rblib.joinm()

  #[Internal] --------------------------
  def _acq_permission(self):
    u"""Private (非公開)"""
    return self.__rblib.acq_permission()

  #[Internal] --------------------------
  def _rel_permission(self):
    u"""Private (非公開)"""
    return self.__rblib.rel_permission()

  #[Internal] --------------------------
  def ioctrl(self, wordno, dataL, maskL, dataH, maskH):
    u"""Private (非公開)"""
    return self.__rblib.ioctrl(wordno, dataL, maskL, dataH, maskH)

  #[Internal] --------------------------
  def _syssts(self, typ):
    u"""Private (非公開)"""
    return self.__rblib.syssts(typ)

  #[Internal] --------------------------
  def _sysctrl(self, ctrlid, arg):
    u"""Private (非公開)"""
    return self.__rblib.sysctrl(ctrlid, arg)

  #[Internal] --------------------------
  def _set_log_level(self, level):
    u"""Private (非公開)"""
    return self.__rblib.set_log_level(level)

  #[Internal] --------------------------
  ## ポーズ用フック
  def _pause_hook(self):
    u"""Private (非公開)"""
    if not self.__sysstatus._ev_pause.is_set() or self.__sysstatus._teachMode:
      #  print "pause_hook: event not set"
      return False

    if PRINT_PAUSE_STATE:
      print "IN PAUSE HOOK"
    if self.__sysstatus._f_servo_off_during_pause:
      if self.__sysstatus._f_restore_pos_before_continue:
        jnt = self.getjnt()
      self.svoff()

    if self.__sysstatus._f_no_pause_during_moving:
      # wait until stop
      self._joinm()

    # Enter pause state
    with self.__sysstatus.pause_lock:
      self.__sysstatus._cur_pause_status |= 0x04 # PS_IN_HOOK
      self.__write_app_status(4,-1,self.__sysstatus._cur_pause_status)
      if PRINT_PAUSE_STATE:
        print "pause in hook"

    # release permission if debug mode
    pause_debug = 0
    if self.__sysstatus._f_no_pause_during_moving:
      pause_debug = int(shm_read(0x0308,1)) & 0x01
      if pause_debug != 0:
        cur_param = self.getmotionparam()
        cur_async = self.__rblib.asyncm(0)[1]
        self._rel_permission()

    # wait continue and svon
    self.__sysstatus._ev_continue.clear()
    while not self.__sysstatus._ev_continue.is_set() or self.svstat() != 1:
      if self.svstat() != 1 and self.__sysstatus._ev_continue.is_set():
        self.__sysstatus._ev_continue.clear()  # svon first
      # stop during pause
      if self.__sysstatus._ev_stop.is_set() and self.__sysstatus._enbErr == True:
        time.sleep(0.5)
        with self.__sysstatus.pause_lock:
          self.__sysstatus._cur_pause_status &= ~0x04 # PS_OUT_HOOK
          self.__write_app_status(0,-1,self.__sysstatus._cur_pause_status)
        if PRINT_PAUSE_STATE:
          print "stop in hook"
        self.last_exit_code = 16  # NERR(16)
        if self.__sysstatus._f_enb_interrupt_stop_in_pause:
          raise Robot_stop
        else:
          self.exit(0)
      # emo during pause
      if self.__sysstatus._ev_emo.is_set() and self.__sysstatus._enbErr == True:
        time.sleep(0.5)
        with self.__sysstatus.pause_lock:
          self.__sysstatus._cur_pause_status &= ~0x04 # PS_OUT_HOOK
          self.__write_app_status(0,-1,self.__sysstatus._cur_pause_status)
        if PRINT_PAUSE_STATE:
          print "emo in hook"
        self.last_exit_code = 13  # NERR(13)
        if self.__sysstatus._f_enb_interrupt_emo_in_pause:
          raise Robot_emo
        else:
          self.exit(0)
      time.sleep(0.5)

    # Begin to continue
    if pause_debug != 0:
      self.__rblib.acq_permission()
      self.motionparam(cur_param)
      self.__rblib.asyncm(cur_async)

    self.__sysstatus._cur_pause_status &= ~0x04 # PS_OUT_HOOK
    self.__write_app_status(0,-1,self.__sysstatus._cur_pause_status)
    if PRINT_PAUSE_STATE:
      print "continue in hook"
    self.__sysstatus._ev_pause.clear()
    self.__sysstatus._ev_continue.clear()

    if (self.__sysstatus._f_servo_off_during_pause and
      self.__sysstatus._f_restore_pos_before_continue and pause_debug == 0):
      self.move(jnt)
    #print "end pause"

    return True

  #[Internal] --------------------------
  def _internal_hook(self,user_call=False):
    u"""Private (非公開)"""
    if self.api_thread_id != thread.get_ident():
      liberr_sub(4,25, "i611Robot._internal_hook")
      self.__sysstatus._fatal_error = self.__sysstatus.EID_INVALIDTHREAD
      self.__sysstatus._ev_fatal_error.set()
      return

    if self.__sysstatus._enbErr == False:
      return
    if not self.__isopened:
      self.__cause_fatal_exit(6) # NERR(6)

    # 致命的エラー発生
    if self.__sysstatus._ev_fatal_error.is_set():
      if self.__sysstatus._power_off != 0:
        raise Robot_poweroff
      self.abort()
      time.sleep(0.5)
      if self.__sysstatus._fatal_error == self.__sysstatus.EID_ABSLOST:
        self.__cause_fatal_exit(7) # NERR(7)
      elif self.__sysstatus._fatal_error == self.__sysstatus.EID_INVALIDMODE:
        self.__cause_fatal_exit(5) # NERR(5)
      elif self.__sysstatus._fatal_error == self.__sysstatus.EID_HW_ERROR:
        self.__cause_fatal_exit(99,9) # CERR(99)
      elif self.__sysstatus._fatal_error == self.__sysstatus.EID_SW_ERROR:
        self.__cause_fatal_exit(99)
      elif self.__sysstatus._fatal_error == self.__sysstatus.EID_INVALIDTHREAD:
        self.__cause_fatal_exit(21) # NERR(21)
      else:
        raise Robot_fatalerror

    # 例外発生
    # 力覚検知を優先して発行する
    if self.__sysstatus._ev_fs_detect.is_set():
      time.sleep(0.5)
      self.last_exit_code = 51 # NERR(51)
      raise Robot_forcesensor

    if self.__sysstatus._ev_stop.is_set() and self.__sysstatus._enbErr == True:
      time.sleep(0.5)
      self.last_exit_code = 16  # NERR(16)
      if self.__sysstatus._f_enb_interrupt_stop:
        raise Robot_stop
      else:
        self.exit(0)

    if self.__sysstatus._ev_emo.is_set():
      time.sleep(0.5)
      self.last_exit_code = 13 # NERR(13)
      #print "raise emo"
      if self.__sysstatus._f_enb_interrupt_emo:
        raise Robot_emo
      else:
        self.exit(0)

    # 一時停止判定
    if not user_call and self.__sysstatus._f_enable_only_user_hook:
      pass # pause is disable
    else:
      self._pause_hook()


  #[Public] ############################
  # 一時停止および減速停止割り込みが発生する場所に埋め込む
  def user_hook(self):
    u"""Pauses a robot program by being placed where you want （ロボットプログラムに割り込む場所を設定する）

    Args:
      None

    Return:
      None

    Note:
      * Roto program can be paused or interrupted in motion command or user_hook, not in the other APIs like time.sleep().
        So user_hook must be called constantly during the other APIs.
      * （ロボット制御コマンド以外のpython処理中は、非常停止や減速停止などの割り込み（例外発生）が
        効きませんので、そのような場所では定期的にこの関数を呼ぶようにしてください。）

    **Example**::

      ...
      rb.user_hook() # program can be paused here （この位置でプログラムを一時停止させる。）
      ...

    """
    self._internal_hook(True)


  #[Public] ############################
  # メインスレッドでtime.sleepの代わりに呼ぶ
  def sleep(self,sec):
    u"""Wait for specified seconds. （指定した時間の処理を一時停止する）

    Args:
      sec(float): duration[s] （秒数(s)）

    Return:
      None

    Note:
      * program cannot be paused or interrupted in time.sleep(). So use i611Robot.sleep() instead.
      * （Python のsleep 関数は、この一時停止中に非常停止スイッチが押されても非常停止の例外を発生させるこ
        とができませんが、このメソッドを使うことで、スリープ中でもロボット関係の例外を発生させることができます。）
      * In order to raise Robot_emo() with Emergency sw, configuration by enable_interrupt is needed.
      * （Robot_emo() クラスを有効にするため、あらかじめ enable_interrupt() を記述してください。）

    **Example**::

      try.
        # Sleep for 5s （指定した秒数スリープする。）
        rb.sleep( sec=5 )
        ...
      except Robot_emo # Handler for Emergency switch. （非常停止SW 押下イベントハンドラ（復帰不能））
        # Error process for Emergency switch. （必要なエラー処理( 終了処理) を記載する）

    """
    if self.api_thread_id != thread.get_ident():
      if time is None or self.__sysstatus._ev_fatal_error.is_set():
        raise Robot_exception
      ## 例外ではなく、通常のsleepを呼ぶ
      time.sleep(sec)
      ## liberr_sub(4,24, "i611Robot.sleep")
      ## self.__sysstatus._fatal_error = self.__sysstatus.EID_INVALIDTHREAD
      ## self.__sysstatus._ev_fatal_error.set()
      return

    max_sleep_time = 0.025  # 25 ms
    target_time = time.time() + sec
    while True:
      self._internal_hook()
      cur_time = time.time()
      if cur_time >= target_time:
        break
      time.sleep(max_sleep_time)


  #[Public] ############################
  # 一時停止等の動作を設定する。
  # user_hook以外での一時停止を禁止する。（例外は発生する。）
  # 一時停止中に、サーボ電源を落とす。
  # 一時停止後の再開時に、姿勢を一時停止前に戻す。
  # 動作コマンド実行中は一時停止しない。
  def set_behavior(self, only_hook=False, servo_off=False,
                   restore_position=False, no_pause=False):
    u"""Specifies the behaviors upon motion pause （一時停止の動作（振る舞い）を設定する）

    Args:
      only_hook(bool): Prohibits any pause instructions except for user_hook() （user_hook() でのみ一時停止を可能にする）

        * True: Enable （有効）
        * False: Disable[Default] （無効（初期値））

      servo_off(bool): Powers off the servo during pause(1) （一時停止時にサーボをOFF にする(1)）

        * True: Enable （有効）
        * False: Disable[Default] （無効（初期値））

      restore_position(bool): Restores the (pre-pause) posture upon operation resume following a pause. （一時停止後の再開時に位置を一時停止前に戻す）

        * True: Enable （有効）
        * False: Disable[Default] （無効（初期値））

      no_pause(bool):Don't pause during motion. （一時停止を動作の区切りのみで行う）

        * True: Enable （有効）
        * False: Disable[Default] （無効（初期値））

    Return:
      None

    Note:
      * 1) To resume motion, turn on Servo power then input run signal. （動作の再開は、サーボをON してから実行(run) の入力の順に行います。）
      * 2) Fix the gap before restore motion when robot shift in pause.  （一時停止中にサーボをOFF して再度ON した時に位置がずれた場合にも、一時停止前の位置に戻ってから動作を再開することができます。）
      * 3) If robot is paused during motion, robot resume the position and then continue motion in spite of this settings. （ 動作中に一時停止した場合は、この設定にかかわらず元の位置に戻ってから再開します。）

    **Example**::

      # Restore positin before restart motion. （一時停止後の再開時に、姿勢を一時停止前に戻す）
      rb.set_behavior( only_hook=False, servo_off=False, restore_position=True, no_pause=True )

    """
    if not self.__isopened:
      self.__cause_fatal_exit(6) # NERR(6)
    liblog("i611Robot.set_behavior",only_hook,servo_off,restore_position,no_pause)
    self.__sysstatus._f_enable_only_user_hook = only_hook
    self.__sysstatus._f_servo_off_during_pause = servo_off
    self.__sysstatus._f_restore_pos_before_continue = restore_position
    self.__sysstatus._f_no_pause_during_moving = no_pause

  #[Public] ############################
  # 例外発生イベントをリセットする
  def release_stopevent(self):
    u"""Clears an exception-raised event such as deceleration stop. （発生中の例外イベントをリセットする）

    Args:
      None

    Return:
      None

    **Example**::

      try:
        … # Robot motion （動作）
      except Robot_stop:
        rb.release_stopevent()
        … # e.g. Evacuate arm （退避動作など）

    """
    if not self.__isopened:
      self.__cause_fatal_exit(6) # NERR(6)
    liblog("i611Robot.release_stopevent")
    self.last_exit_code = 0
    self.__sysstatus._ev_stop.clear()
    #print "ev_stop clear"
    self.__sysstatus._ev_fs_detect.clear()
    self.__sysstatus._ev_emo.clear()
    self.__sysstatus._ev_pause.clear()
    self.__sysstatus._ev_continue.clear()


  #[Public] ############################
  # ユーザーエラー（クリティカル・ノーマル）を発生させる。
  def cause_user_error(self, code, critical=False):
    u"""Raises a user-defined error （ユーザ定義エラーを発生させます。）

    Args:
      code(int): Error ID （エラーID）

        * Range （設定範囲）: 1 - 99

      critical(bool): Error type[Default:False] （エラー種別（初期値: False））

        * True: A user-defined fatal error has occurred. （ユーザー定義エラー（致命的）発生）
        * False: A user-defined error has occurred (the initial setting) （ユーザー定義エラー発生）

    Return:
      None

    **Example**::

      # Raises a user-defined error No.19 （ユーザ定義エラー（エラーID：19）を発生させる場合）
      rb.cause_user_error( 19, False )

      # Raises a user-defined fatal error No.01 （ユーザ定義エラー致命的（ エラーID：1）を発生させる場合)
      rb.cause_user_error( 1, True )

    """
    if not self.__isopened:
      self.__cause_fatal_exit(6) # NERR(6)
    liblog("i611Robot.cause_user_error", code,critical)
    print "cause_user_error(%d)" % code
    if code<1 or code>99:
      code = 99
    mode = 10 if critical==False else 11
    self.__write_app_status(mode,code)
    os._exit(1)


  #[Public] ############################
  # 減速停止、非常停止時の割り込み発生を有効・無効化する。
  def enable_interrupt(self, eid, enable):
    u"""Enables or disables interrupt of deceleration stop or emergency stop （減速停止と非常停止の例外の発生を設定する）

    Args:
      eid(int): EventID（イベントID）

        - 0: raise an exception at deceleration stop input during motion （動作中の減速停止入力時の例外発生）
        - 1: raise an exception upon E-stop input during motion （動作中の非常停止入力時の例外発生）
        - 2: raise an exception upon deceleration stop input during pause （一時停止中の減速停止入力時の例外発生）
        - 3: raise an exception upon E-stop input during pause （一時停止中の非常停止入力時の例外発生）

      enable(bool): Changes the enable/disable settings. （例外の発生）

        - True: Enable （有効）
        - False: Disable （無効）

    Return:
      bool: Returns True if it succeed, otherwise an exception is raised. （成功したときにTrueが返り、それ以外は例外が発生します。）

    **Example**::

      # Ex1: Enable exception at deceleration stop input during motion
      # 例1：動作中の減速停止入力時の例外発生を有効にする
      rb.enable_interrupt( 0, True )

      # Ex2: Enable exception upon E-stop input during motion
      # 例2：動作中の非常停止入力時の例外発生を有効にする
      rb.enable_interrupt( 1, True )

      # Ex3: Disable exception upon deceleration stop input during pause
      # 例3：一時停止中の減速停止入力時の例外発生を無効にする
      rb.enable_interrupt( 2, False )

      # Ex4: Disable exception upon E-stop input during pause
      # 例4：一時停止中の非常停止入力時の例外発生を無効にする
      rb.enable_interrupt( 3, False )

    """
    if not self.__isopened:
      self.__cause_fatal_exit(6) # NERR(6)
    liblog("i611Robot.enable_interrupt", eid,enable)
    ret = False
    if eid == 0: # 動作中の減速停止
      self.__sysstatus._f_enb_interrupt_stop = bool(enable)
      ret = True
    elif eid == 1: # 動作中の非常停止
      self.__sysstatus._f_enb_interrupt_emo = bool(enable)
      ret = True
    elif eid == 2: # 一時停止中の減速停止
      self.__sysstatus._f_enb_interrupt_stop_in_pause = bool(enable)
      ret = True
    elif eid == 3: # 一時停止中の非常停止
      self.__sysstatus._f_enb_interrupt_emo_in_pause = bool(enable)
      ret = True
    return ret


  #[Internal] --------------------------
  # システムエラーのメッセージ文字列を取得
  @staticmethod
  def _get_syserr_msg(st,code):
    u"""Private (非公開)"""
    return get_syserr_msg(st,code)


  #############################


  #[Internal] --------------------------
  def detect_forcesensor(self, err=0):
    u"""Private (非公開)"""
    liblog("i611Robot.detect_forcesensor", err)
    if err==1:
      self.__cause_fatal_exception(50)  # NERR(50)
    elif err==2:
      self.__cause_fatal_exception(52)  # NERR(52)
    else:
      self.__sysstatus._ev_fs_detect.set()
      self.abort()

  #[Internal] --------------------------
  def is_detected_forcesensor(self):
    u"""Private (非公開)"""
    return self.__sysstatus._ev_fs_detect.is_set()

  #[Internal] --------------------------
  # [ACCL] 加速抑制On/Off
  def accel_limit_enable(self, enable, min_thresh=-1):
    u"""Private (非公開)"""
    liblog("i611Robot.accel_limit_enable",enable, min_thresh)
    self.accel_limit = enable
    if min_thresh>-1:
      self.min_ticks = float(min_thresh)

  #[Internal] --------------------------
  # [ACCL] PTP用速度変換
  def convert_accel_ptp(self, _x, _y, _z, _rz, _ry, _rx, _posture,
    _rbcoord, _multiturn, _ik_solver_option, _speed, _acct, _dacct):
    u"""Private (非公開)"""
    if _use_mt:
      ret = self.__rblib.ptpplan_mt( _x, _y, _z, _rz, _ry, _rx, _posture, _rbcoord,
        _multiturn, _ik_solver_option, _speed,_acct,_dacct)
    else:
      ret = self.__rblib.ptpplan( _x, _y, _z, _rz, _ry, _rx, _posture, _rbcoord,
        _speed,_acct,_dacct)
    if ret[0]==False:
        return self._error(ret[1],ret[2])
    #print "ticks=%.2f,%.2f,%.2f" % (ret[1], ret[2], ret[3])
    total_ticks = ret[1]+ret[2]+ret[3]
    _speed = self.calc_speed(total_ticks,_speed,ret[8:14])
    return _speed

  #[Internal] --------------------------
  # [ACCL] CP用速度変換
  def convert_accel_cp(self, _x, _y, _z, _rz, _ry, _rx, _posture,
    _rbcoord, _speed, _acct, _dacct):
    u"""Private (非公開)"""
    ret = self.__rblib.cpplan( _x, _y, _z, _rz, _ry, _rx, _posture,
    _rbcoord, _speed, _acct, _dacct)
    if ret[0]==False:
        return self._error(ret[1],ret[2])
    total_ticks = ret[1]+ret[2]+ret[3]
    _speed = self.calc_speed(total_ticks,_speed,ret[8:14])
    return _speed

  #[Internal] --------------------------
  # [ACCL] OPTCP用速度変換
  def convert_accel_optcp(self, _x, _y, _z, _rz, _ry, _rx, _posture,
    _rbcoord, _speed, _acct, _dacct):
    u"""Private (非公開)"""
    ret = self.__rblib.optcpplan( _x, _y, _z, _rz, _ry, _rx, _posture, _rbcoord,
      _speed,_acct,_dacct)
    if ret[0]==False:
        return self._error(ret[1],ret[2])
    #print "ticks=%.2f,%.2f,%.2f" % (ret[1], ret[2], ret[3])
    total_ticks = ret[1]+ret[2]+ret[3]
    _speed = self.calc_speed(total_ticks,_speed,ret[8:14])
    return _speed

  #[Internal] --------------------------
  # [ACCL]速度変換関数
  # total_ticks: 総Tick数
  # orig_speed: 返還前の速度 (%)
  # rate_list: 各軸の動作量比(6軸分)
  # 戻り値：適切なspeedを返す。
  def calc_speed(self, total_ticks, orig_speed, rate_list):
    u"""Private (非公開)"""
    if total_ticks  >= self.min_ticks:
      return orig_speed
    _ = rate_list # avoid warning
    new_speed = ( total_ticks / self.min_ticks ) * ( total_ticks / self.min_ticks ) * orig_speed
    print "calc_speed(%.2f,%.2f)=%.2f" % (total_ticks,orig_speed,new_speed)
    return new_speed

  #[Public] ############################
  # (相対)Joint動作
  #pram [dj1,dj2,dj3,dj4,dj5,dj6]
  def reljntmove(self, *arg1, **arg2):
    u"""Instructs relative PTP motion by Joint offset（Joint座標系で相対動作をする）

    Args:
      dj1(float): J1 offset[deg] （J1軸の相対移動量[deg]）
      dj2(float): J2 offset[deg] （J2軸の相対移動量[deg]）
      dj3(float): J3 offset[deg] （J3軸の相対移動量[deg]）
      dj4(float): J4 offset[deg] （J4軸の相対移動量[deg]）
      dj5(float): J5 offset[deg] （J5軸の相対移動量[deg]）
      dj6(float): J6 offset[deg] （J6軸の相対移動量[deg]）

    Return:
      None

    **Example**::

      J1 = Joint( 45, 45, -45, -45, 90, 0 )

      # Set motion parameters （動作パラメータを設定する）
      m=MotionParam( jnt_speed=10, lin_speed=70, overlap=30 )
      rb.motionparam( m )
      ...
      rb.move( J1 )

      # PTP motion to J1=+35[deg] position. （Joint 座標系でJ1 を35deg オフセットした位置に移動する）
      rb.reljntmove( dj1=35 )

    """
    p = i611_common._args([0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              ['dj1', 'dj2', 'dj3', 'dj4', 'dj5', 'dj6'],
              [float, float, float, float, float, float],
              *arg1, **arg2)
    if p[0] == False:
      return self._error(4, p[1],"reljntmove")
    else:
      _speed = self.__mp2[1]
      _acct = self.__mp2[2]
      _dacct = self.__mp2[3]
      if not self.__sysstatus._f_no_pause_during_moving:
        self._internal_hook() ###[HOOK]###
      res = self.__rblib.jntrmove(p[1], p[2], p[3], p[4], p[5], p[6],_speed,_acct,_dacct)
      liblog(("%d = i611Robot.reljntmove" % res[1]), p[1], p[2], p[3], p[4], p[5], p[6],_speed,_acct,_dacct)
      self._internal_hook() ###[HOOK]###
      if res[0] == False:
        return self._error(res[1], res[2])
      else:
        return res

  #[Public] ############################
  # (相対)直線補間動作
  #pram [dx,dy,dz,drz,dry,drx]
  def relline(self, *arg1, **arg2):
    u"""Instructs relative line motion by X-Y offset （直交座標系で相対直線補間動作をする）

    Args:
      dx(float): X offset[mm] （X軸方向のオフセット量[mm]）
      dy(float): Y offset[mm] （Y軸方向のオフセット量[mm]）
      dz(float): Z offset[mm] （Z軸方向のオフセット量[mm]）
      drz(float): Rz offset[deg] （Z軸まわりのオフセット量[deg]）
      dry(float): Ry offset[deg] （Y軸まわりのオフセット量[deg]）
      drx(float): Rx offset[deg] （X軸まわりのオフセット量[deg]）

    Return:
      None

    **Example**::

      P10 = Position( 95, -280, 240, 154, 80, -114 )

      # set motion parameters （動作パラメータを設定する）
      m=MotionParam( jnt_speed=10, lin_speed=70, overlap=30 )
      rb.motionparam( m )
      ...
      rb.move(P10)

      # line motion to X=+15 position. （直交座標系でX 軸方向に15mm オフセットした位置に移動する）
      rb.relline( dx=15 )

    """
    p = i611_common._args([0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              ['dx', 'dy', 'dz', 'drz', 'dry', 'drx'],
              [float, float, float, float, float, float],
              *arg1, **arg2)
    if p[0] == False:
      #self._mcslog(106)
      return self._error(4, p[1],"relline")
    else:
      _speed = self.__mp2[0]
      _acct = self.__mp2[2]
      _dacct = self.__mp2[3]
      if not self.__sysstatus._f_no_pause_during_moving:
        self._internal_hook() ###[HOOK]###
      res = self.__rblib.cprmove(p[1], p[2], p[3], p[4], p[5], p[6],_speed,_acct,_dacct)
      liblog(("%d = i611Robot.relline" % res[1]),p[1], p[2], p[3], p[4], p[5], p[6],_speed,_acct,_dacct)
      self._internal_hook() ###[HOOK]###
      if res[0] == False:
        return self._error(res[1], res[2])
      else:
        return res

  #[Public] ############################
  # 座標を文字列に変換(丸め込み)したときの近接MT値を算出
  # 戻り値：MT値
  # pos = 丸め込む前のPosition座標
  # str_x,str_y,str_z,str_rz,str_ry,str_rx = 文字列に丸め込んだPosition座標
  def adjust_mt(self, pos, str_x, str_y, str_z, str_rz, str_ry, str_rx):
      u"""Fix crossover counter value error caused by conversion from float to string. (Position 型座標値を文字列に変換する時のCC 値を補正する）

      Args:
        pos(Position): Original position （元になるPosition 座標）
        str_x(str): string of x value （x 座標の文字列）
        str_y(str): string of y value （y 座標の文字列）
        str_z(str): string of z value （z 座標の文字列）
        str_rz(str): string of rz （rz 座標の文字列）
        str_ry(str): string of ry （ry 座標の文字列）
        str_rx(str): string of rz （rz 座標の文字列）

      Return:
        long: Fixed crossover counter （補正したCC値）

      .. hint::
        * When float value is converted to string, a value contains a small error.
          So if position is on the border of crossover counter, a small shift
          of position causes the change of crossover counter.
          Therefore, this API calculates the appropriate crossover counter
          from original position data and string expression.
        * （float数を文字列に変換すると、丸め込み誤差が発生することがあります。
          したがって、座標がCC値が変化する境界値付近だったときは、
          丸め込み誤差によりCC値との関係が矛盾する可能性があるため、CC値も合わせて
          補正することで矛盾を解消します。）

      **Example**::

        rb = i611Robot()
        rb.open()
        pos = rb.getpos()
        pos_value = pos.position()
        pos_str = [str (round(x,2) ) for x in pos_value[0:6] ]
        new_mt = rb.adjust_mt(pos, pos_str[0], pos_str[1], pos_str[2], pos_str[3], pos_str[4], pos_str[5])
        pos_str += [str (pos_value[7]), "0x%06X" % new_mt]
        print "Position String:%s" % pos_str


      """
      # 型チェックはここでは敢えて行わない。（別の例外が発生）
      b = pos.position(True)
      res = self.__rblib.getmt(b[0], b[1], b[2], b[3], b[4], b[5], b[7], 1, b[8], str_x, str_y, str_z, str_rz, str_ry, str_rx)
      liblog(("%d = i611Robot.adjust_mt" % res[1]),b[0], b[1], b[2], b[3], b[4], b[5], b[7], 1, b[8],str_x, str_y, str_z, str_rz, str_ry, str_rx)
      if res[0] == False:
        return self._error(res[1], res[2])
      else:
        return int(res[1])

  #[Public] ############################
  #pram [pos1, pos2, orientation]
  def arcmove(self, pos1, pos2, orientation):
    u"""Instructs arc motion method. （円弧補間動作）

    Args:
      pos1(Position): position of via point. (経由点ロボット座標値)
      pos2(Position): position of destination. (終点ロボット座標値)
      orientation(long): behavior of pose. (姿勢の取り扱い)
      
        * 0:Keep current pose in X-Y coordinate. (始点姿勢をワールド座標系上で維持)
        * 1:Keep current pose in tool coordinate. (始点姿勢を軌道座標系上で維持)
        * 2:Intereporate pose between current position and destination. (軌道座標系上で始点姿勢から終点姿勢へSLERP補間)

    Return:
      bool: Returns True if it succeed, otherwise an exception is raised. （成功したときにTrueが返り、それ以外は例外が発生します。）

    **Example**::

      pos_home = Joint(2.77, -19.19, 108.03,   0.49, 101.15,   -180.84)
      pos1 = Position(229.97,-392.93, 552.25,  90.00,  -0.00,-180.00, 7)
      pos2 = Position(-129.92,-392.85, 552.24,  90.00,  -0.00, 180.00, 7)

      rb.move(pos_home)
      rb.arcmove( pos1, pos2, 0 )
      ...
      rb.close()

    """
    if not isinstance(pos1,Position):
      return self._error(4,4,"arcmove")
    if not isinstance(pos2,Position):
      print type(pos2)
      return self._error(4,4,"arcmove")
    orientation = long(orientation)

    _speed = self.__mp2[0]
    _acct = self.__mp2[2]
    _dacct = self.__mp2[3]
    if not self.__sysstatus._f_no_pause_during_moving:
      self._internal_hook() ###[HOOK]###

    _x1, _y1, _z1, _rz1, _ry1, _rx1, _, _ps1 = pos1.position()[0:8]
    _x2, _y2, _z2, _rz2, _ry2, _rx2, _, _ps2 = pos2.position()[0:8]
    res = self.__rblib.arcmove(
      _x1, _y1, _z1, _rz1, _ry1, _rx1, _ps1, 1,
      _x2, _y2, _z2, _rz2, _ry2, _rx2, _ps2, 1,
      _speed,_acct,_dacct,orientation)

    if not self.__sysstatus._teachMode:
      liblog(("%d = i611Robot.arcmove" % res[1]),pos1.pos, pos2.pos, _speed,_acct,_dacct, orientation)
    self._internal_hook() ###[HOOK]###
    if res[0] == False:
      return self._error(res[1], res[2])
    else:
      return res

  #[Public] ############################
  #pram [pos1, pos2, orientation]
  def circlemove(self, pos1, pos2, orientation):
    u"""Instructs circle motion method. （円補間動作）

    Args:
      pos1(Position): position of via point 1. (経由点1ロボット座標値)
      pos2(Position): position of via point 2. (経由点1ロボット座標値)
      orientation(long): behavior of pose. (姿勢の取り扱い)

        * 0:Keep current pose in X-Y coordinate. (始点姿勢をワールド座標系上で維持)
        * 1:Keep current pose in tool coordinate. (始点姿勢を軌道座標系上で維持)

    Return:
      bool: Returns True if it succeed, otherwise an exception is raised. （成功したときにTrueが返り、それ以外は例外が発生します。）

    **Example**::

      pos_home = Joint(2.77, -19.19, 108.03,   0.49, 101.15,   -180.84)
      pos1 = Position(229.97,-392.93, 552.25,  90.00,  -0.00,-180.00, 7)
      pos2 = Position(-129.92,-392.85, 552.24,  90.00,  -0.00, 180.00, 7)

      rb.move(pos_home)
      rb.circlemove( pos1, pos2, 0 )
      ...
      rb.close()

    """
    if not isinstance(pos1,Position):
      return self._error(4,4,"circlemove")
    if not isinstance(pos2,Position):
      print type(pos2)
      return self._error(4,4,"circlemove")
    orientation = long(orientation)

    _speed = self.__mp2[0]
    _acct = self.__mp2[2]
    _dacct = self.__mp2[3]
    if not self.__sysstatus._f_no_pause_during_moving:
      self._internal_hook() ###[HOOK]###

    _x1, _y1, _z1, _rz1, _ry1, _rx1, _, _ps1 = pos1.position()[0:8]
    _x2, _y2, _z2, _rz2, _ry2, _rx2, _, _ps2 = pos2.position()[0:8]
    res = self.__rblib.cirmove(
      _x1, _y1, _z1, _rz1, _ry1, _rx1, _ps1, 1,
      _x2, _y2, _z2, _rz2, _ry2, _rx2, _ps2, 1,
      _speed,_acct,_dacct,orientation)

    if not self.__sysstatus._teachMode:
      liblog(("%d = i611Robot.circlemove" % res[1]),pos1.pos, pos2.pos, _speed,_acct,_dacct, orientation)
    self._internal_hook() ###[HOOK]###
    if res[0] == False:
      return self._error(res[1], res[2])
    else:
      return res

  #[Public] ############################
  # システムステータス＆エラーコード取得
  # 戻り値：(status, error)
  # status = 1:init, 2:ready, 3:inc, 4:teach, 6:run, 10:NormalErr, 11:CriticalErr
  #          12:UserNormalErr, 13:UserCriticalErr
  # error = 0～99
  @staticmethod
  def get_system_status():
      u"""Gets system status and error id. （システム状態とエラーID を取得する）

      Args:
        None

      Return:
        list: [ status, err_id ]

          * status(int): System status （システム状態）

            - 1: Initializing （起動中）
            - 2: Ready （待機状態）
            - 3: ABS lost （ABS消失状態）
            - 4: Teaching （ティーチング中）
            - 5: Jog mode （JOG操作モード）
            - 6: Running program （ロボットプログラム実行中）
            - 10: System error （システム定義エラー発生中）
            - 11: Fatal system error （システム定義エラー（致命的）発生中）
            - 12: User defined system error （ユーザ定義エラー発生中）
            - 13: User defined fatal system error （ユーザ定義エラー（致命的）発生中）
          * err_id(int): error id （エラーID）


      Note:
        * Error id is two-digit decimal. if no error occurred, error id is 0.
        * エラーIDは、エラー発生時（statusが10以上）のときの2桁のエラー値となります。正常時は、0となります。


      .. hint::
        * Since this API is static method, i611Robot instance is not required.
        * （このメソッドはスタティックメソッドです。i611Robot クラスのインスタンス定義をしなくても呼び出せます。）


      **Example**::

        # Call api as static method. （スタティックメソッドとしての呼び出し）
        status, err_id = i611Robot.get_system_status()

      """
      reg = 0x0308
      reg_val = int(shm_read(reg,1))
      sts = (reg_val >> 4) & 0x0F
      eid = (reg_val >> 8) & 0x0FF
      return sts, eid

  #[Public] ############################
  # モデル名、シリアルナンバー取得
  # 戻り値：(modelname, serialnumber)
  @staticmethod
  def get_hw_info():
      u"""Get model name and serial number. （機種名とシリアル番号を取得する）

      Args:
        None

      Return:
        list: [ model_name, serial_number ]

          * model_name(str): model name （モデル名）
          * serial_number(str): Serial number （シリアル番号）

      .. hint::
        * Since this API is static method, i611Robot instance is not required.
        * （このメソッドはスタティックメソッドです。i611Robot クラスのインスタンス定義をしなくても呼び出せます。）

      **Example**::

        model, serial = i611Robt.get_hw_info()

      """
      reg = 0x2C00
      return shm_read(reg,2).split(',')


  #[Public] ############################
  # システムポート(8個)の状態を取得（RobSys.req_mcmdと同等）
  # 戻り値：( running, svon, emo, hw_error, sw_error, abs_lost, in_pause, error, (reserved)
  @staticmethod
  def get_system_port():
    u"""Get status of system port. （システムポートの状態を取得する）

    Args:
      None

    Return:
      list: [running, svon, emo, hw_error, sw_error, abs_lost, in_pause, error, rsv]

        * running(int): if program is running or not. （ロボットプログラム状態）

          - 1: Running （実行中)
          - 0: Stopped （停止中）
        * svon(int): if servo power is turned on or off. （サーボ状態）

          - 1: Servo On （サーボON 中）
          - 0: Servo Off （サーボOFF 中）
        * emo(int): if emergency button is pressed or not. （非常停止状態）

          - 1: Emergency （非常停止中）
          - 0: None
        * hw_error(int): if fatal system error occurs. （システム定義エラー( 致命的) 状態）

          - 1: Fatal system error occurs （致命的システムエラー発生中）
          - 0: None
        * sw_error(int): if system error occurs. （システムエラー状態）

          - 1: System error occcurs エラー発生中
          - 0: None
        * abs_lost(int): if ABS losts. （ABS 消失状態）

          - 1: ABS lost （ABS 消失中）
          - 0: None
        * in_pause(int): if program is paused. （一時停止状態）

          - 1: Paused （一時停止中）
          - 0: None
        * error(int): if either system error or fatal system error occurs. （システムエラーまたは致命的システムエラー状態）

          - 1: Error occurs （エラー発生中）
          - 0: None
        * rsv(int): (予約)

    .. hint::
        * Since this API is static method, i611Robot instance is not required.
        * （このメソッドはスタティックメソッドです。i611Robot クラスのインスタンス定義をしなくても呼び出せます。）

    **Example**::

      # Check each status （個別にシステムの状態を確認する）
      port = rb.get_system_por()
      running = port[0]
      svon = port[1]
      emo = port[2]
      hw_error = port[3]
      sw_error = port[4]
      abs_lost = port[5]
      in_pause = port[6]
      error = port[7]

    """
    REG_HW_INFO = 0x0300
    REG_SW_INFO = 0x0308
    reg_val = {}
    reg_val[REG_HW_INFO] = int(shm_read(REG_HW_INFO,1))
    reg_val[REG_SW_INFO] = int(shm_read(REG_SW_INFO,1))

    running  = (reg_val[REG_SW_INFO] >> 0) & 0x01    ## 0:ユーザープログラム動作中
    svon     = (reg_val[REG_HW_INFO] >> 0) & 0x01    ## 1:サーボ電源
    emo      = (reg_val[REG_HW_INFO] >> 1) & 0x01    ## 2:非常停止状態
    hw_error = (reg_val[REG_HW_INFO] >> 2) & 0x01    ## 3:クリティカルエラー発生中
    sw_error = (reg_val[REG_SW_INFO] >> 1) & 0x01    ## 4:システムエラー発生中
    abs_lost = (reg_val[REG_HW_INFO] >> 3) & 0x01    ## 5:ABS消失中
    in_pause = (reg_val[REG_SW_INFO] >> 2) & 0x01    ## 6:一時停止中
    error    = (reg_val[REG_SW_INFO] >> 3) & 0x01    ## 7:エラー発生中
    return [running, svon, emo, hw_error, sw_error, abs_lost, in_pause, error, 0]


  #[Public] ############################
  """ checkReady()  :  ロボットプログラムが実行可能かどうかチェック
  戻り値：
  0... ユーザープログラム実行OK
  1... 非常停止中
  2... サーボOFF
  3... 自動モード外(JOG接続中など)
  4... 制御権が取れない
  5... その他エラー発生中"""
  @staticmethod
  def check_ready(msg=False):
    u"""Check if robot program is runnable. （ロボットが自動運転できるかを確認する）

    Args:
      msg(bool): True when the result should be output to console. （Trueのときは、標準出力にメッセージを出力する。）

    Return:
      int: Result [not zero means not runnable] （確認結果 (0以外は自動運転不可））

        * 0: Runnable. （ロボットプログラム実行可能（自動運転可能））
        * 1: Not runnable because of emergency. （非常停止中）
        * 2: Not runnable because of servo off. （サーボOFF）
        * 3: Not runnable because of invalid mode like jog mode. （自動モードではない(JOG スティック接続中など)）
        * 4: Not runnable because of permission. （操作権が取れない）
        * 5: Other error happens （その他エラー発生中）

    .. hint::
      * This method is to confirm if i611Robot instance can be used or not.
      *  （i611Robot クラスのインスタンスが使えるかどうかを事前に確認するメソッドです。）
      * When this API returns non-zero, i611Robot constructor or open raises Exception.
      *  （戻り値が'0' 以外の場合、ロボットは動作できません。i611Robot クラスのコンストラクタまたはopen() の実行時に例外が発生します。）
      * このメソッドで事前に状態を確認すると、例外の発生を回避できます。
      * Since this API is static method, i611Robot instance is not required.
      * （このメソッドはスタティックメソッドです。i611Robot クラスのインスタンス定義をしなくても呼び出せます。）

    **Example**::

      res = i611Robot.check_ready()
      if res != 0:
        # some process like print message or output I/O （メッセージ表示や外部出力による通知など）
        sys.exit(0)

    """
    # jog check
    if not os.path.exists("/tmp/auto_ready"):
      if msg==True:
          print "Not auto mode"
      return 3

    # port check
    __rblib = rblib.Robot("127.0.0.1",12345)
    __rblib.open()
    res = __rblib.ioctrl(128, 0, 0xffffffff, 0, 0xffffffff)
    if res[0]==False:
      __rblib.close()
      if msg==True:
        print "Unable to connect control manager"
      return 5

    if res[1] & 0x02:
      __rblib.close()
      if msg==True:
        print "Emergency stop"
      return 1

    if not res[1] & 0x01:
      __rblib.close()
      if msg==True:
        print "Servo off"
      return 2

    # check permission
    res = __rblib.acq_permission()
    if res[0]==False:
      __rblib.close()
      if msg==True:
        print "Robot is controlled by another program"
      return 4
    __rblib.rel_permission()
    __rblib.close()

    # all ready!
    if msg==True:
      print "Ready"
    return 0

## 入れ子になっているので、この位置で呼ばざるを得ない・・・
#pylint: disable=C0413
from teachdata import Teachdata

#eof

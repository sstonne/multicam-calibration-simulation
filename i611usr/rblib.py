#!/usr/bin/env python
# -*- coding: utf-8 -*-


import sys
import time
import json
import socket
import math
import threading

def version_rblib():
    return [0, 3, 1, 0]

class Robot(object):
    # 受信バッファサイズ
    _N_RCVBUF = 1024

    # コマンドコード
    _NOP = 1
    _SVSW = 2
    _PLSMOVE = 3
    _MTRMOVE = 4
    _JNTMOVE = 5
    _PTPMOVE = 6
    _CPMOVE = 7
    _SET_TOOL = 8
    _CHANGE_TOOL = 9
    _ASYNCM = 10
    _PASSM = 11
    _OVERLAP = 12
    _MARK = 13
    _JMARK = 14
    _IOCTRL = 15
    _ZONE = 16
    _J2R = 17
    _R2J = 18
    _SYSSTS = 19
    _TRMOVE = 20
    _ABORTM = 21
    _JOINM = 22
    _SLSPEED = 23
    _ACQ_PERMISSION = 24
    _REL_PERMISSION = 25
    _SET_MDO = 26
    _ENABLE_MDO = 27
    _DISABLE_MDO = 28
    _PMARK = 29
    _VERSION = 30
    _ENCRST = 31
    _SAVEPARAMS = 32
    _CALCPLSOFFSET = 33
    _SET_LOG_LEVEL = 34
    _FSCTRL = 35
    _PTPPLAN = 36
    _CPPLAN = 37
    _PTPPLAN_W_SP = 38
    _CPPLAN_W_SP = 39
    _SYSCTRL = 40
    _OPTCPMOVE = 41
    _OPTCPPLAN = 42
    _PTPMOVE_MT = 43
    _MARK_MT = 44
    _J2R_MT = 45
    _R2J_MT = 46
    _PTPPLAN_MT = 47
    _PTPPLAN_W_SP_MT = 48
    _JNTRMOVE = 49
    _JNTRMOVE_WO_CHK= 50
    _CPRMOVE = 51
    _CPRPLAN = 52
    _CPRPLAN_W_SP = 53
    _SUSPENDM = 54
    _RESUMEM = 55
    _GETMT = 56
    _RELBRK = 57
    _CLPBRK = 58
    _ARCMOVE = 59
    _CIRMOVE = 60

    def __init__(self, host, port):
        self._host = host   # '127.0.0.1'
        self._port = port   # 12345
        self._lock = threading.Lock()
        self._sock = None

    def __del__(self):
        self.close()

    def open(self):
        if self._sock is None:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.connect((self._host, self._port))
            if self.version()[0] == False:
                raise Exception("Error! Unable to open rblib.")
            else:
                return True
        else:
            return True

    def close(self):
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def chkRes(self, cmdid, buf):
        #print buf
        if buf == '':
            print 'chkRes buff is empty'
            return [False, 99, 1]  # トランザクションエラー　バッファが空

        jsonobj = json.loads(buf)
        if cmdid != jsonobj['cmd']:
            #print 'error chkRes', cmdid, buf
            buf = self._sock.recv(self._N_RCVBUF)
            jsonobj = json.loads(buf)
            #print 'retry', buf
            if cmdid != jsonobj['cmd']:
                print 'chkRes cmd ID error'
                return [False, 99, 1]  # トランザクションエラー　コマンドIDの不一致

        return jsonobj['results']

    # NOP
    def nop(self):
        params = {'cmd':Robot._NOP,'params':[]}
        print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                #print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)
        return self.chkRes(params['cmd'], buf)

    def svctrl(self, sw):
        params = {'cmd':Robot._SVSW,'params':[int(sw)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                #print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    #  引数   各モータパルス値 ax1, ax2, ax3, ax4, ax5, ax6 [pulse]
    #        speed=速度 [%], acct=加速時間 [s], dacct=減速時間 [s]
    #  戻り値 動作ID
    def plsmove(self, ax1, ax2, ax3, ax4, ax5, ax6, speed, acct, dacct):
        params = {'cmd':Robot._PLSMOVE,'params':[[long(ax1),long(ax2),long(ax3),\
            long(ax4),long(ax5),long(ax6)],float(speed),float(acct),float(dacct)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    #  引数   各モータ減速機出力軸角度 ax1, ax2, ax3, ax4, ax5, ax6 [deg]
    #        speed=速度 [%], acct=加速時間 [s], dacct=減速時間 [s]
    #  戻り値 動作ID
    def mtrmove(self, ax1, ax2, ax3, ax4, ax5, ax6, speed, acct, dacct):
        params = {'cmd':Robot._MTRMOVE,'params':[[math.radians(ax1), \
            math.radians(ax2),math.radians(ax3),math.radians(ax4), \
            math.radians(ax5),math.radians(ax6)],float(speed),float(acct),float(dacct)]}

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    #  引数   ジョイント座標値 ax1, ax2, ax3, ax4, ax5, ax6 [deg]
    #        speed=速度 [%], acct=加速時間 [s], dacct=減速時間 [s]
    #  戻り値 動作ID
    def jntmove(self, ax1, ax2, ax3, ax4, ax5, ax6, speed, acct, dacct):
        params = {'cmd':Robot._JNTMOVE,'params':[[math.radians(ax1),math.radians(ax2),\
        math.radians(ax3),math.radians(ax4),math.radians(ax5),math.radians(ax6)],\
        float(speed),float(acct),float(dacct)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    #  引数   ロボット座標値 x,y,z [mm]
    #                     rz.ry,rx, [deg]
    #                     posture,rbcoord
    #        speed=速度 [%], acct=加速時間 [s], dacct=減速時間 [s]
    #  戻り値 動作ID
    def ptpmove(self, x, y, z, rz, ry, rx, posture, rbcoord, speed, acct, dacct):
        params = {'cmd':Robot._PTPMOVE,'params':[[x/1000.0,y/1000.0,z/1000.0,\
            math.radians(rz),math.radians(ry),math.radians(rx), long(posture), \
            long(rbcoord)],float(speed),float(acct),float(dacct)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    #  引数   ロボット座標値 x,y,z [mm]
    #                     rz.ry,rx, [deg]
    #                     posture,rbcoord
    #        speed=速度 [mm/s], acct=加速時間 [s], dacct=減速時間 [s]
    #  戻り値 動作ID
    def cpmove(self, x, y, z, rz, ry, rx, posture, rbcoord, speed, acct, dacct):
        params = {'cmd':Robot._CPMOVE,'params':[[x/1000.0,y/1000.0,z/1000.0,\
        math.radians(rz),math.radians(ry),math.radians(rx), long(posture), \
        long(rbcoord)],float(speed)/1000.0,float(acct),float(dacct)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    #  引数   ロボット座標値 x,y,z [mm]
    #                     rz.ry,rx, [deg]
    #                     posture,rbcoord
    #        speed=速度 [%], acct=加速時間 [s], dacct=減速時間 [s]
    #  戻り値 なし
    def optcpmove(self, x, y, z, rz, ry, rx, posture, rbcoord, speed, acct, dacct):
        params = {'cmd':Robot._OPTCPMOVE,'params':[[x/1000.0,y/1000.0,z/1000.0,
        math.radians(rz),math.radians(ry),math.radians(rx), long(posture),
        long(rbcoord)],float(speed),float(acct),float(dacct)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    #  引数　　tid ツール番号(1-8)
    #      　 アーム端座標系上でのオフセット値 offx,offy,offz [mm]
    #                                    offrz.offry,offrx, [deg]
    #  戻り値 なし
    def settool(self, tid, offx, offy, offz, offrz, offry, offrx):
        params = {'cmd':Robot._SET_TOOL,'params':[int(tid),
                                                  offx/1000.0,
                                                  offy/1000.0,
                                                  offz/1000.0,
                                                  math.radians(offrz),
                                                  math.radians(offry),
                                                  math.radians(offrx)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    #  引数   tid ツール番号　(0-8, 0=ツールオフセット解除)
    #  戻り値 なし
    def changetool(self, tid):
        params = {'cmd':Robot._CHANGE_TOOL,'params':[int(tid)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    #  引数  0=読み出しのみ 1=非同期動作ON  2=非同期動作OFF
    #  戻り値 直前の設定値
    def asyncm(self, sw):
        params = {'cmd':Robot._ASYNCM,'params':[int(sw)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    #  引数  0=読み出しのみ 1=パス動作ON  2=パス動作OFF
    #  戻り値 直前の設定値
    def passm(self, sw):
        params = {'cmd':Robot._PASSM,'params':[int(sw)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    #  引数  0.0=オーバーラップ解除 >0.0=次動作開始距離[mm]
    #  戻り値 無し
    def overlap(self, distance):
        params = {'cmd':Robot._OVERLAP,'params':[float(distance)/1000.0]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    #  引数  無し
    #  戻り値 現在位置のロボット座標での値 [mm],[deg]
    #        x,y,z,rz,ry,rx,posture, rbcoord
    def mark(self):
        params = {'cmd':Robot._MARK,'params':[]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        retlist = self.chkRes(params['cmd'], buf)
        if retlist[0] == True:
            # mm->m
            retlist[1] = float(retlist[1]) * 1000.0
            retlist[2] = float(retlist[2]) * 1000.0
            retlist[3] = float(retlist[3]) * 1000.0
            # rad->deg
            retlist[4] = math.degrees(float(retlist[4]))
            retlist[5] = math.degrees(float(retlist[5]))
            retlist[6] = math.degrees(float(retlist[6]))
        return retlist

    #  引数  無し
    #  戻り値 現在位置のジョイント座標での値[deg]
    #        ax1,ax2,ax3,ax4,ax5,ax6
    def jmark(self):
        params = {'cmd':Robot._JMARK,'params':[]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        retlist = self.chkRes(params['cmd'], buf)
        if retlist[0] == True:
            # rad->deg
            retlist[1] = math.degrees(retlist[1])
            retlist[2] = math.degrees(retlist[2])
            retlist[3] = math.degrees(retlist[3])
            retlist[4] = math.degrees(retlist[4])
            retlist[5] = math.degrees(retlist[5])
            retlist[6] = math.degrees(retlist[6])
        return retlist

    #  引数   wordno(32bit単位、0オリジン)
    #        dataL, dataH　設定データ（各々32bit)
    #        maskL, maskH　変更しないbitの指定（各々32bit、1で変更しない)
    #  戻り値 32bitx2 変更前のデータ
    def ioctrl(self, wordno, dataL, maskL, dataH, maskH):
        params = {'cmd':Robot._IOCTRL,'params':[wordno, dataL, maskL, dataH, maskH]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        retlist = self.chkRes(params['cmd'], buf)
        return retlist

    # 位置決め判定範囲
    #  引数   位置決め判定範囲[pulse]
    #  戻り値 直前の設定値
    def zone(self, pulse):
        params = {'cmd':Robot._ZONE,'params':[pulse]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        retlist = self.chkRes(params['cmd'], buf)
        return retlist

    # ジョイント座標からロボット座標の計算（ツールは現在の設定値を使用）
    #  引数   ジョイント座標値 ax1, ax2, ax3, ax4, ax5, ax6 [deg]ロボット座標指定 rbcoord(現状は無視）
    #  戻り値 x,y,z,rz,ry,rx,posture,rbcoord [mm], [deg]
    def j2r(self, ax1, ax2, ax3, ax4, ax5, ax6, rbcoord):
        params = {'cmd':Robot._J2R, 'params':[math.radians(ax1),
                                              math.radians(ax2),
                                              math.radians(ax3),
                                              math.radians(ax4),
                                              math.radians(ax5),
                                              math.radians(ax6),
                                              rbcoord]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        retlist = self.chkRes(params['cmd'], buf)
        if retlist[0] == True:
            # m -> mm
            retlist[1] = float(retlist[1])*1000.0
            retlist[2] = float(retlist[2])*1000.0
            retlist[3] = float(retlist[3])*1000.0
            # rad->deg
            retlist[4] = math.degrees(float(retlist[4]))
            retlist[5] = math.degrees(float(retlist[5]))
            retlist[6] = math.degrees(float(retlist[6]))
        return retlist

    # ロボット座標からジョイント座標の計算（ツールは現在の設定値を使用）
    #  引数   x,y,z,rz,ry,rx,posture,rbcoord [mm], [deg]
    #  戻り値 ax1,ax2,ax3,ax4,ax5,ax6 [deg]
    def r2j(self, x, y, z, rz, ry, rx, posture, rbcoord):
        params = {'cmd':Robot._R2J, 'params':[x/1000.0,
                                              y/1000.0,
                                              z/1000.0,
                                              math.radians(rz),
                                              math.radians(ry),
                                              math.radians(rx),
                                              posture,
                                              rbcoord]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        retlist = self.chkRes(params['cmd'], buf)
        if retlist[0] == True:
            # rad->deg
            retlist[1] = math.degrees(retlist[1])
            retlist[2] = math.degrees(retlist[2])
            retlist[3] = math.degrees(retlist[3])
            retlist[4] = math.degrees(retlist[4])
            retlist[5] = math.degrees(retlist[5])
            retlist[6] = math.degrees(retlist[6])
        return retlist

    # 非常停止状態の読み出し
    #  引数       1:EMS状態　正常(0)/片切れ(1/2)/EMS状態(3)
    #            2:直近の動作の動作ID
    #            3:INCモード軸のビットフィールド
    #            4:ABSロスト軸のビットフィールド
    #            5:動作リクエスト残数（実行中を含む）
    #            6:動作状態　エラーによる動作中断（3）/ホールドによる動作中断（2)/動作中(1)/停止中(0)
    #            7:SPIエラー発生状況
    #            8:サーボ制御状態
    #       0x8000:パルスリストを指定した動作
    #  戻り値 0:非常停止でない 1,2:片切り 3:非常停止中
    def syssts(self, typ):
        params = {'cmd':Robot._SYSSTS, 'params':[int(typ)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        retlist = self.chkRes(params['cmd'], buf)
        return retlist

    # ツール座標系での相対動作
    #  引数   ツール座標京相対値  x,y,z [mm]
    #                         rz.ry,rx, [deg]
    #        speed=速度 [mm/s], acct=加速時間 [s], dacct=減速時間 [s]
    #  戻り値 動作ID
    def trmove(self, x, y, z, rz, ry, rx, speed, acct, dacct):
        params = {'cmd':Robot._TRMOVE,'params':[[x/1000.0,y/1000.0,z/1000.0,math.radians(rz),\
        math.radians(ry),math.radians(rx)], float(speed)/1000.0,float(acct),float(dacct)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    # 動作の中断
    #  引数   なし
    #  戻り値 中断した動作の動作ID
    def abortm(self):
        params = {'cmd':Robot._ABORTM,'params':[]}
        # print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    #  戻り値 最後の動作の動作ID
    def joinm(self):
        params = {'cmd':Robot._JOINM,'params':[]}
        # print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    # 姿勢補間速度の設定
    #  引数   姿勢補間速度[%]  7/4時点で100%=2PI/8 rad/s
    #  戻り値 直前の設定値
    def slspeed(self, spd):
        params = {'cmd':Robot._SLSPEED,'params':[float(spd)]}
        # print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        retlist = self.chkRes(params['cmd'], buf)
        return retlist

    # ロボット操作権の取得
    #  引数   なし
    #  戻り値 なし
    def acq_permission(self):
        params = {'cmd':Robot._ACQ_PERMISSION,'params':[]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        retlist = self.chkRes(params['cmd'], buf)
        return retlist

    # ロボット操作権の開放
    #  引数   なし
    #  戻り値 なし
    def rel_permission(self):
        params = {'cmd':Robot._REL_PERMISSION,'params':[]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        retlist = self.chkRes(params['cmd'], buf)
        return retlist

    # MDO機能の設定
    #  引数   : mdoid   : MDO管理番号(1-8)
    #          portno   : 出力ポート番号
    #          value    : 短絡(0), 開放(1)
    #          kind     : 始点から一定距離離れた(1)、終点から一定距離内に入った(2)
    #          distance : double  : 距離[mm]（>=0.0）
    #  戻り値 なし
    def set_mdo(self, mdoid, portno, value, kind, distance):
        params = {'cmd':Robot._SET_MDO,'params':[int(mdoid),int(portno),\
            int(value),int(kind),float(distance)/1000.0]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        retlist = self.chkRes(params['cmd'], buf)
        return retlist

    # MDO機能有効化
    #  引数   : bitfield : 有効にするmdoidに関連したbitを立てる
    #  戻り値 なし
    def enable_mdo(self, bitfield):
        params = {'cmd':Robot._ENABLE_MDO,'params':[int(bitfield)]}
        print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        retlist = self.chkRes(params['cmd'], buf)
        return retlist

    # MDO機能無効化
    #  引数   : bitfield : 無効にするmdoidに関連したbitを立てる
    #  戻り値 なし
    def disable_mdo(self, bitfield):
        params = {'cmd':Robot._DISABLE_MDO,'params':[int(bitfield)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        retlist = self.chkRes(params['cmd'], buf)
        return retlist

    #  引数   sw 取得パルスの種類指定
    #            1:pcurp　　　　　（フィードバックパルス）
    #            2:pcmdp　　　　　（指令パルス）
    #            3:curpos.pls　　（現在位置パルス=フィードバックパルス）
    #            4:lastgoal.pls　（ラストゴールのパルス）
    #  戻り値
    #        0    : 取得できるパルス種類を示す文字列
    #        1..4 : パルス値   ax1,ax2,ax3,ax4,ax5,ax6
    def pmark(self, sw):
        params = {'cmd':Robot._PMARK,'params':[int(sw)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        retlist = self.chkRes(params['cmd'], buf)
        return retlist

    #  引数   なし
    #  戻り値　I メジャーバージョン番号
    #        I マイナーバージョン番号
    #        I パッチバージョン番号
    #        I ビルド番号
    #        S ビルド日付（文字列）
    def version(self):
        params = {'cmd':Robot._VERSION,'params':[]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        retlist = self.chkRes(params['cmd'], buf)
        return retlist

    # エンコーダリセット
    #  引数   リセットする軸のビットフィールド
    #  戻り値 なし
    def encrst(self, bitfield):
        params = {'cmd':Robot._ENCRST,'params':[int(bitfield)]}
        # print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        retlist = self.chkRes(params['cmd'], buf)
        return retlist

    # パラメータファイルの保存
    #  引数   なし
    #  戻り値 なし
    def saveparams(self):
        params = {'cmd':Robot._SAVEPARAMS,'params':[]}
        # print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        retlist = self.chkRes(params['cmd'], buf)
        return retlist

    # パルスオフセットの計算
    #  引数   typ  オフセットタイプ　1:エンコーダ−＞機械原点オフセット（工場原点復帰）
    #                            2:芯出しオフセット
    #                            3:エンコーダ−＞機械原点オフセット（ユーザー原点復帰）
    #        bitfield オフセットパルスを取得する軸のビットフィールド表記
    #  戻り値 なし
    def calcplsoffset(self, typ, bitfield):
        params = {'cmd':Robot._CALCPLSOFFSET,'params':[int(typ), int(bitfield)]}
        # print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        retlist = self.chkRes(params['cmd'], buf)
        return retlist

    # ログレベルの設定
    #  引数   設定するログレベル
    #          -1:ログレベルを変更せずに現在の設定値を取得
    #           0:DEBUG以上のログを出力
    #           1:INFO以上のログを出力
    #           2:NOTICE以上のログを出力
    #           3:WARNING以上のログを出力
    #           4:ERR以上のログを出力
    #  戻り値 コマンド実行前のログレベル
    def set_log_level(self, level):
        params = {'cmd':Robot._SET_LOG_LEVEL, 'params':[int(level)]}
        # print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        retlist = self.chkRes(params['cmd'], buf)
        return retlist

    # 力覚センサの制御
    #  引数   コマンドコード
    #           1:中点設定
    #           2:センサ値読み出し（実数）
    #           3:センサ値読み出し（生データ）
    #           4:変換ゲイン読み出し（実数）
    #           5:変換ゲイン読み出し（文字列）
    #  戻り値 コマンド実行前のログレベル
    def fsctrl(self, cmd):
        params = {'cmd':Robot._FSCTRL, 'params':[int(cmd)]}
        # print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        retlist = self.chkRes(params['cmd'], buf)
        return retlist

    # PTP動作プランパラメータの取得
    #  引数   終点ロボット座標値 x,y,z [mm]
    #                        rz.ry,rx, [deg]
    #                        posture,rbcoord
    #        speed=速度 [%], acct=加速時間 [s], dacct=減速時間 [s]
    #  戻り値　nacc     加速Tick数
    #         ncvl     等速Tick数
    #         ndec     減速Tick数
    #         maxdst   主軸動作量 [pulse]
    #         maxvel   最大動作速度 [pulse/tick]
    #         accel    加速度 [pulse/tick/tick]
    #         decel    減速度 [pulse/tick/tick]
    #         rate[]   主軸を基準とした各軸の動作量比
    def ptpplan(self, ex, ey, ez, erz, ery, erx, eposture, erbcoord, speed, acct, dacct):
        params = {'cmd':Robot._PTPPLAN,'params':[[ex/1000.0,ey/1000.0,ez/1000.0,\
            math.radians(erz),math.radians(ery),math.radians(erx), long(eposture), \
            long(erbcoord)], float(speed),float(acct),float(dacct)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)


    # 直線補間動作プランパラメータの取得
    #  引数   終点ロボット座標値 x,y,z [mm]
    #                        rz.ry,rx, [deg]
    #                        posture,rbcoord
    #        speed=速度 [%], acct=加速時間 [s], dacct=減速時間 [s]
    #  戻り値　nacc     加速Tick数
    #         ncvl     等速Tick数
    #         ndec     減速Tick数
    #         maxdst   動作距離 [m]
    #         maxvel   最大動作速度 [m/tick]
    #         accel    加速度 [m/tick/tick]
    #         decel    減速度 [m/tick/tick]
    #         rate[]   主軸を基準とした各軸の動作量比
    def cpplan(self, ex, ey, ez, erz, ery, erx, eposture, erbcoord, speed, acct, dacct):
        params = {'cmd':Robot._CPPLAN,'params':[[ex/1000.0,ey/1000.0,ez/1000.0,\
        math.radians(erz),math.radians(ery),math.radians(erx), long(eposture), \
        long(erbcoord)], float(speed)/1000.0,float(acct),float(dacct)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)


    # PTP動作プランパラメータの取得
    #  引数   始点ロボット座標値 x,y,z [mm]
    #                        rz.ry,rx, [deg]
    #                        posture,rbcoord
    #        終点ロボット座標値 x,y,z [mm]
    #                        rz.ry,rx, [deg]
    #                        posture,rbcoord
    #        speed=速度 [%], acct=加速時間 [s], dacct=減速時間 [s]
    #  戻り値　nacc     加速Tick数
    #         ncvl     等速Tick数
    #         ndec     減速Tick数
    #         maxdst   主軸動作量 [pulse]
    #         maxvel   最大動作速度 [pulse/tick]
    #         accel    加速度 [pulse/tick/tick]
    #         decel    減速度 [pulse/tick/tick]
    #         rate[]   主軸を基準とした各軸の動作量比
    def ptpplan_w_sp(self, sx, sy, sz, srz, sry, srx, sposture, srbcoord, ex, ey, \
        ez, erz, ery, erx, eposture, erbcoord, speed, acct, dacct):
        params = {'cmd':Robot._PTPPLAN_W_SP,'params':[[sx/1000.0,sy/1000.0, \
            sz/1000.0,math.radians(srz),math.radians(sry),math.radians(srx), \
            long(sposture), long(srbcoord)],[ex/1000.0,ey/1000.0,ez/1000.0, \
            math.radians(erz),math.radians(ery),math.radians(erx), long(eposture), \
            long(erbcoord)], float(speed),float(acct),float(dacct)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    # 直線補間動作プランパラメータの取得
    #  引数   始点ロボット座標値 x,y,z [mm]
    #                        rz.ry,rx, [deg]
    #                        posture,rbcoord
    #        終点ロボット座標値 x,y,z [mm]
    #                        rz.ry,rx, [deg]
    #                        posture,rbcoord
    #        speed=速度 [%], acct=加速時間 [s], dacct=減速時間 [s]
    #  戻り値　nacc     加速Tick数
    #         ncvl     等速Tick数
    #         ndec     減速Tick数
    #         maxdst   動作距離 [m]
    #         maxvel   最大動作速度 [m/tick]
    #         accel    加速度 [m/tick/tick]
    #         decel    減速度 [m/tick/tick]
    #         rate[]   主軸を基準とした各軸の動作量比
    def cpplan_w_sp(self, sx, sy, sz, srz, sry, srx, sposture, srbcoord, ex, ey, ez, \
        erz, ery, erx, eposture, erbcoord, speed, acct, dacct):
        params = {'cmd':Robot._CPPLAN_W_SP,'params':[[sx/1000.0,sy/1000.0,sz/1000.0,\
        math.radians(srz),math.radians(sry),math.radians(srx), long(sposture), \
        long(srbcoord)],[ex/1000.0,ey/1000.0,ez/1000.0,math.radians(erz),\
        math.radians(ery),math.radians(erx), long(eposture), long(erbcoord)], \
        float(speed)/1000.0,float(acct),float(dacct)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    # 最適直線補間動作プランパラメータの取得
    #  引数   終点ロボット座標値 x,y,z [mm]
    #                        rz.ry,rx, [deg]
    #                        posture,rbcoord
    #        speed=速度 [%], acct=加速時間 [s], dacct=減速時間 [s]
    #  戻り値　nacc     加速Tick数
    #         ncvl     等速Tick数
    #         ndec     減速Tick数
    #         maxdst   主軸動作量 [pulse]
    #         maxvel   最大動作速度 [pulse/tick]
    #         accel    加速度 [pulse/tick/tick]
    #         decel    減速度 [pulse/tick/tick]
    #         rate[]   主軸を基準とした各軸の動作量比
    def optcpplan(self, ex, ey, ez, erz, ery, erx, eposture, erbcoord, speed, acct, dacct):
        params = {'cmd':Robot._OPTCPPLAN,'params':[[ex/1000.0,ey/1000.0,ez/1000.0,
            math.radians(erz),math.radians(ery),math.radians(erx), long(eposture),
            long(erbcoord)], float(speed),float(acct),float(dacct)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    # native制御
    #  引数   制御番号   ctrlid      1:7SEG制御権の設定
    #                         0x8000:強制エラーステート遷移
    #        　　　　　　arg   パラメータ
    #                         ctrlid=     1 : 0=native7SEG制御無効 1=native7SEG制御有効
    #                         ctrlid=0x8000 : 0xdead=エラーステートに遷移 else=何もしない
    #                                       : 0xeeee=強制エラー（@DEBUG)
    #                                       : 0x8001=タスク実行時間max値のクリア
    #  戻り値　なし
    def sysctrl(self, ctrlid, arg):
        params = {'cmd':Robot._SYSCTRL,'params':[int(ctrlid), int(arg)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    # 多回転情報を考慮したPTP動作
    #  引数   ロボット座標値 x,y,z [mm]
    #                     rz.ry,rx, [deg]
    #                     posture,rbcoord,multiturn,ik_solver_option
    #        speed=速度 [%], acct=加速時間 [s], dacct=減速時間 [s]
    #  戻り値 動作ID
    #
    #  NOTE1
    #        multiturn:多回転情報
    #         1軸4bit*8軸分。最上位バイトFFで多回転データ無効
    #
    #                -900   -540   -180    180    540    900
    #         turnNo. -+------+------+------+------+------+-
    #            2                                 ●------○
    #            1                          ●------○
    #            0                   ●------○
    #           -1(F)         ●------○
    #           -2(E)  ●------○
    #　　　　　　
    #  NOTE2
    #        nativeIFにはmultiturnの他、ik_solver_optionが追加されています。
    #        全軸multiturnで指定した回転位置にするには0x11111111を指定します。
    #
    #　　　　　　0:指定位置（なければ現在位置）に対して最近値。【従来互換】
    #　　　　　　1:multiturnで指定した位置。
    #　　　　　　2:指定位置（なければ現在位置）に対して＋方向の最近値。
    #　　　　　　0:指定位置（なければ現在位置）に対してー方向の最近値。
    #　　　　　　
    def ptpmove_mt(self, x, y, z, rz, ry, rx, posture, rbcoord, multiturn, ik_solver_option, speed, acct, dacct):
        if (multiturn & 0xFF000000) == 0xFF000000:
            ik_solver_option = 0x00000000

        params = {'cmd':Robot._PTPMOVE_MT,'params':[[x/1000.0,y/1000.0,z/1000.0,\
            math.radians(rz),math.radians(ry),math.radians(rx), long(posture), \
            long(rbcoord),long(multiturn), ik_solver_option], float(speed),float(acct),float(dacct)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    # 多回転情報を含めた現在位置のロボット座標値を取得する。
    #  引数  無し
    #  戻り値 現在位置のロボット座標での値 [mm],[deg]
    #        x,y,z,rz,ry,rx,posture, rbcoord, multiturn
    def mark_mt(self):
        params = {'cmd':Robot._MARK_MT,'params':[]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        retlist = self.chkRes(params['cmd'], buf)
        if retlist[0] == True:
            # mm->m
            retlist[1] = float(retlist[1]) * 1000.0
            retlist[2] = float(retlist[2]) * 1000.0
            retlist[3] = float(retlist[3]) * 1000.0
            # rad->deg
            retlist[4] = math.degrees(float(retlist[4]))
            retlist[5] = math.degrees(float(retlist[5]))
            retlist[6] = math.degrees(float(retlist[6]))
            retlist[9] = long(retlist[9])
        return retlist

    # ジョイント座標からロボット座標の計算（ツールは現在の設定値を使用）
    #  引数   ジョイント座標値 ax1, ax2, ax3, ax4, ax5, ax6 [deg]ロボット座標指定 rbcoord(現状は無視）
    #  戻り値 x,y,z[mm],rz,ry,rx[deg],posture,rbcoord, multiturn,singular_info,area_info
    def j2r_mt(self, ax1, ax2, ax3, ax4, ax5, ax6, rbcoord):
        params = {'cmd':Robot._J2R_MT, 'params':[math.radians(ax1),
                                              math.radians(ax2),
                                              math.radians(ax3),
                                              math.radians(ax4),
                                              math.radians(ax5),
                                              math.radians(ax6),
                                              rbcoord]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        retlist = self.chkRes(params['cmd'], buf)
        if retlist[0] == True:
            # m -> mm
            retlist[1] = float(retlist[1])*1000.0
            retlist[2] = float(retlist[2])*1000.0
            retlist[3] = float(retlist[3])*1000.0
            # rad->deg
            retlist[4] = math.degrees(float(retlist[4]))
            retlist[5] = math.degrees(float(retlist[5]))
            retlist[6] = math.degrees(float(retlist[6]))
            retlist[9] = long(retlist[9])
        return retlist

    # ロボット座標からジョイント座標の計算（ツールは現在の設定値を使用）
    #  引数   x,y,z[mm],rz,ry,rx[deg],posture,rbcoord,multiturn,ik_solver_option
    #  戻り値 ax1,ax2,ax3,ax4,ax5,ax6 [deg], singular_info, area_info
    def r2j_mt(self, x, y, z, rz, ry, rx, posture, rbcoord, multiturn,ik_solver_option):
        if (multiturn & 0xFF000000) == 0xFF000000:
            ik_solver_option = 0x00000000
        params = {'cmd':Robot._R2J_MT, 'params':[x/1000.0,
                                              y/1000.0,
                                              z/1000.0,
                                              math.radians(rz),
                                              math.radians(ry),
                                              math.radians(rx),
                                              posture,
                                              rbcoord,
                                              multiturn,
                                              ik_solver_option]}

         #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        retlist = self.chkRes(params['cmd'], buf)
        if retlist[0] == True:
            # rad->deg
            retlist[1] = math.degrees(retlist[1])
            retlist[2] = math.degrees(retlist[2])
            retlist[3] = math.degrees(retlist[3])
            retlist[4] = math.degrees(retlist[4])
            retlist[5] = math.degrees(retlist[5])
            retlist[6] = math.degrees(retlist[6])
        return retlist

    # PTP動作プランパラメータの取得
    #  引数   終点ロボット座標値 x,y,z [mm]
    #                        rz.ry,rx, [deg]
    #                        posture,rbcoord,multiturn,ik_solver_option
    #        speed=速度 [%], acct=加速時間 [s], dacct=減速時間 [s]
    #  戻り値　nacc     加速Tick数
    #         ncvl     等速Tick数
    #         ndec     減速Tick数
    #         maxdst   主軸動作量 [pulse]
    #         maxvel   最大動作速度 [pulse/tick]
    #         accel    加速度 [pulse/tick/tick]
    #         decel    減速度 [pulse/tick/tick]
    #         rate[]   主軸を基準とした各軸の動作量比
    def ptpplan_mt(self, ex, ey, ez, erz, ery, erx, eposture, erbcoord, emultiturn, ik_solver_option, speed, acct, dacct):
        if (emultiturn & 0xFF000000) == 0xFF000000:
            ik_solver_option = 0x00000000
        params = {'cmd':Robot._PTPPLAN_MT,'params':[[ex/1000.0,ey/1000.0,ez/1000.0,\
            math.radians(erz),math.radians(ery),math.radians(erx), long(eposture), \
            long(erbcoord), long(emultiturn), long(ik_solver_option)], \
            float(speed),float(acct),float(dacct)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    # PTP動作プランパラメータの取得
    #  引数   始点ロボット座標値 x,y,z [mm]
    #                        rz.ry,rx, [deg]
    #                        posture,rbcoord,multiturn,ik_solver_option
    #        終点ロボット座標値 x,y,z [mm]
    #                        rz.ry,rx, [deg]
    #                        posture,rbcoord,multiturn,ik_solver_option
    #        speed=速度 [%], acct=加速時間 [s], dacct=減速時間 [s]
    #  戻り値　nacc     加速Tick数
    #         ncvl     等速Tick数
    #         ndec     減速Tick数
    #         maxdst   主軸動作量 [pulse]
    #         maxvel   最大動作速度 [pulse/tick]
    #         accel    加速度 [pulse/tick/tick]
    #         decel    減速度 [pulse/tick/tick]
    #         rate[]   主軸を基準とした各軸の動作量比
    def ptpplan_w_sp_mt(self, sx, sy, sz, srz, sry, srx, sposture, srbcoord, smultiturn, sik_solver_option, ex, ey, \
                        ez, erz, ery, erx, eposture, erbcoord, emultiturn, eik_solver_option, speed, acct, dacct):
        if (smultiturn & 0xFF000000) == 0xFF000000:
            sik_solver_option = 0x00000000
        if (emultiturn & 0xFF000000) == 0xFF000000:
            eik_solver_option = 0x00000000
        params = {'cmd':Robot._PTPPLAN_W_SP_MT,'params':[[sx/1000.0,sy/1000.0, \
            sz/1000.0,math.radians(srz),math.radians(sry),math.radians(srx), \
            long(sposture), long(srbcoord), long(smultiturn), long(sik_solver_option)],[ex/1000.0,ey/1000.0,ez/1000.0, \
            math.radians(erz),math.radians(ery),math.radians(erx), long(eposture), \
            long(erbcoord), long(emultiturn), long(eik_solver_option)], float(speed),float(acct),float(dacct)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    #  機能　 ジョイント相対動作量を指定した動作（ソフトリミットチェック有り）
    #  引数   ジョイント座標相対動作量 ax1, ax2, ax3, ax4, ax5, ax6 [deg]
    #        speed=速度 [%], acct=加速時間 [s], dacct=減速時間 [s]
    #  戻り値 動作ID
    def jntrmove(self, ax1, ax2, ax3, ax4, ax5, ax6, speed, acct, dacct):
        params = {'cmd':Robot._JNTRMOVE,'params':[[math.radians(ax1),math.radians(ax2),\
        math.radians(ax3),math.radians(ax4),math.radians(ax5),math.radians(ax6)],\
        float(speed),float(acct),float(dacct)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    #  機能   ジョイント相対動作量を指定した動作（ソフトリミットチェックなし）
    #  引数   ジョイント座標相対動作量 ax1, ax2, ax3, ax4, ax5, ax6 [deg]
    #        speed=速度 [%], acct=加速時間 [s], dacct=減速時間 [s]
    #  戻り値 動作ID
    def jntrmove_wo_chk(self, ax1, ax2, ax3, ax4, ax5, ax6, speed, acct, dacct):
        params = {'cmd':Robot._JNTRMOVE_WO_CHK,'params':[[math.radians(ax1),math.radians(ax2),\
        math.radians(ax3),math.radians(ax4),math.radians(ax5),math.radians(ax6)],\
        float(speed),float(acct),float(dacct)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    #  機能   グローバル直交座標系での相対動作
    #  引数   グローバル直交座標系上での相対動作量  x,y,z [mm]
    #                                        rz.ry,rx, [deg]
    #        speed=速度 [mm/s], acct=加速時間 [s], dacct=減速時間 [s]
    #  戻り値 動作ID
    def cprmove(self, x, y, z, rz, ry, rx, speed, acct, dacct):
        params = {'cmd':Robot._CPRMOVE,'params':[[x/1000.0,y/1000.0,z/1000.0,math.radians(rz),\
        math.radians(ry),math.radians(rx)], speed/1000.0,float(acct),float(dacct)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    # 相対直線補間動作プランパラメータの取得
    #  引数   グローバル座標系での相対動作量 dx,dy,dz [mm]
    #                                  drz,dry,drx, [deg]
    #        speed=速度 [%], acct=加速時間 [s], dacct=減速時間 [s]
    #  戻り値　nacc     加速Tick数
    #         ncvl     等速Tick数
    #         ndec     減速Tick数
    #         maxdst   主軸動作量 [pulse]
    #         maxvel   最大動作速度 [pulse/tick]
    #         accel    加速度 [pulse/tick/tick]
    #         decel    減速度 [pulse/tick/tick]
    #         rate[]   主軸を基準とした各軸の動作量比
    def cprplan(self, dx, dy, dz, drz, dry, drx, speed, acct, dacct):
        params = {'cmd':Robot._CPRPLAN,'params':[[dx/1000.0,\
                                                  dy/1000.0,\
                                                  dz/1000.0,\
                                                  math.radians(drz),\
                                                  math.radians(dry),\
                                                  math.radians(drx),\
                                                 ],\
                                                 speed/1000.0,\
                                                 float(acct),\
                                                 float(dacct)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    # 始点指定相対直線補間動作プランパラメータの取得
    #  引数  グローバル座標系での動作開始点 sx, sy, sz       [mm]
    #                                 drz, dry, drx    [deg]
    #                                 posture          0..7
    #                                 rbcoord          1固定
    #        グローバル座標系での相対動作量 dx,dy,dz [mm]
    #                                  drz,dry,drx, [deg]
    #        speed=速度 [%], acct=加速時間 [s], dacct=減速時間 [s]
    #  戻り値　nacc     加速Tick数
    #         ncvl     等速Tick数
    #         ndec     減速Tick数
    #         maxdst   主軸動作量 [pulse]
    #         maxvel   最大動作速度 [pulse/tick]
    #         accel    加速度 [pulse/tick/tick]
    #         decel    減速度 [pulse/tick/tick]
    #         rate[]   主軸を基準とした各軸の動作量比
    def cprplan_w_sp(self, x, y, z, rz, ry, rx, posture, rbcoord, \
                     dx, dy, dz, drz, dry, drx, speed, acct, dacct):
        params = {'cmd':Robot._CPRPLAN_W_SP,'params':[[x/1000.0,\
                                                       y/1000.0,\
                                                       z/1000.0,\
                                                       math.radians(rz),\
                                                       math.radians(ry),\
                                                       math.radians(rx),\
                                                       long(posture),\
                                                       long(rbcoord),\
                                                      ],\
                                                      [dx/1000.0,\
                                                       dy/1000.0,\
                                                       dz/1000.0,\
                                                       math.radians(drz),\
                                                       math.radians(dry),\
                                                       math.radians(drx)
                                                      ],\
                                                      speed/1000.0,\
                                                      float(acct),\
                                                      float(dacct)]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    # 動作の中断(再開機能有り）
    #  引数   タイムアウト時間 tmout [s], (0.0, 60.0]
    #  戻り値 戻り時の状態
    #          0:RBタスクがサスペンド状態になった
    #          1:RBタスクサスペンド確認タイムアウト
    #          2:サスペンドするロボットタスクが存在しない（制御権を取ったタスクがない）
    #          3:RBタスクで動作コマンドが実行されていない
    #          4:RBタスクはすでにサスペンド状態（多重サスペンド）
    #        中断した動作の動作ID
    def suspendm(self, tmout):
        params = {'cmd':Robot._SUSPENDM,'params':[float(tmout)]}
        # print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    # SUSPENDM下動作の再開
    #  引数   なし
    #  戻り値 最後の動作の動作ID
    def resumem(self):
        params = {'cmd':Robot._RESUMEM,'params':[]}
        # print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    # ベースのジョイント位置に一番近い丸め後のジョイント値のmt値を返す。
    # 丸め後のロボット座標値は、変換誤差をなくす目的で文字列とする。
    #  引数   ベースロボット座標値 bx,by,bz [mm]
    #        　　　　　　        brz.bry,brx, [deg]
    #                     　　　posture,rbcoord,multiturn
    #        丸め後ロボット座標値 str_x,str_y,str_z [mm, string]
    #  　　　　　　　　　　　　　　str_rz,str_ry,str_rx [deg, string]
    #                          * posture, rbcoordはベースと同じ値を使用する
    #  戻り値 丸め後のmt値
    #
    def getmt(self, bx, by, bz, brz, bry, brx, posture, rbcoord, multiturn, str_x, str_y, str_z, str_rz, str_ry, str_rx):
        if (multiturn & 0xFF000000) == 0xFF000000:
            ik_solver_option = 0x00000000

        params = {'cmd':Robot._GETMT,'params':[[bx/1000.0,by/1000.0,bz/1000.0,\
            math.radians(brz),math.radians(bry),math.radians(brx), long(posture), \
            long(rbcoord),long(multiturn)], \
            [str_x, str_y, str_z, str_rz, str_ry, str_rx]]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    # 強制ブレーキ開放
    #  引数   ブレーキ開放する軸のビットフィールド
    #  戻り値 なし
    def relbrk(self, bitfield):
        params = {'cmd':Robot._RELBRK,'params':[int(bitfield)]}
        # print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        retlist = self.chkRes(params['cmd'], buf)
        return retlist

    # 強制ブレーキ
    #  引数   ブレーキする軸のビットフィールド
    #  戻り値 なし
    def clpbrk(self, bitfield):
        params = {'cmd':Robot._CLPBRK,'params':[int(bitfield)]}
        # print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        retlist = self.chkRes(params['cmd'], buf)
        return retlist

    # 円弧補間動作
    #  引数   経由点ロボット座標値 px,py,pz [mm]
    #                          prz.pry,prx, [deg]
    #                          pposture,prbcoord
    #        終点ロボット座標値   ex,ey,ez [mm]
    #                          erz.ery,erx, [deg]
    #                          eposture,erbcoord
    #        speed=速度 [mm/s], acct=加速時間 [s], dacct=減速時間 [s]
    #        orientation = 姿勢の取り扱い（bitfield)
    #                          0:始点姿勢をワールド座標系上で維持
    #                          1:始点姿勢を軌道座標系上で維持
    #                          2:軌道座標系上で始点姿勢から終戦姿勢へSLERP補間
    #  戻り値 動作ID
    def arcmove(self, px, py, pz, prz, pry, prx, pposture, prbcoord,\
                      ex, ey, ez, erz, ery, erx, eposture, erbcoord,\
                      speed, acct, dacct,\
                      orientation):
        params = {'cmd':Robot._ARCMOVE,'params':[\
        [px/1000.0,py/1000.0,pz/1000.0,\
         math.radians(prz),math.radians(pry),math.radians(prx),
         long(pposture),long(prbcoord)],\
        [ex/1000.0,ey/1000.0,ez/1000.0,\
         math.radians(erz),math.radians(ery),math.radians(erx),
         long(eposture),long(erbcoord)],\
        float(speed)/1000.0,float(acct),float(dacct),\
        long(orientation) ]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)

    # 円補間動作
    #  引数   経由点1ロボット座標値 p1x,p1y,p1z [mm]
    #                           p1rz.p1ry,p1rx, [deg]
    #                           p1posture,p1rbcoord
    #        経由点2ロボット座標値 p2x,p2y,p2z [mm]
    #                           p2rz.p2ry,p2rx, [deg]
    #                           p2posture,p2rbcoord
    #        speed=速度 [mm/s], acct=加速時間 [s], dacct=減速時間 [s]
    #        orientation = 姿勢の取り扱い
    #                          0:始点姿勢をワールド座標系上で維持
    #                          1:始点姿勢を軌道座標系上で維持
    #  戻り値 動作ID
    def cirmove(self, p1x, p1y, p1z, p1rz, p1ry, p1rx, p1posture, p1rbcoord,\
                      p2x, p2y, p2z, p2rz, p2ry, p2rx, p2posture, p2rbcoord,\
                      speed, acct, dacct,\
                      orientation):
        params = {'cmd':Robot._CIRMOVE,'params':[\
        [p1x/1000.0,p1y/1000.0,p1z/1000.0,\
         math.radians(p1rz),math.radians(p1ry),math.radians(p1rx),
         long(p1posture),long(p1rbcoord)],\
        [p2x/1000.0,p2y/1000.0,p2z/1000.0,\
         math.radians(p2rz),math.radians(p2ry),math.radians(p2rx),
         long(p2posture),long(p2rbcoord)],\
        float(speed)/1000.0,float(acct),float(dacct),\
        long(orientation) ]}
        #print json.dumps(params)

        with self._lock:
            if self._sock.send(json.dumps(params)) <= 0:
                print "tx error\n"
                return False
            buf = self._sock.recv(self._N_RCVBUF)

        return self.chkRes(params['cmd'], buf)


    ## リモートPC上でデバッグする際の便利関数
    def o_pen(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        return self._sock.connect((self._host, self._port))

if __name__ == '__main__':
    #speed=5.0
    #acct=0.2
    #dacct=0.2
    rb = Robot("i611", 12344)
    rb.open()

    while True:
        if rb.jntmove(45, 0, 0, 0, 0, 0, 5, 1.0, 1.0)[0] != True:
            sys.exit(1)
        time.sleep(1)
        if rb.jntmove(-45, 0, 0, 0, 0, 0, 5, 1.0, 1.0)[0] != True:
            sys.exit(1)
        time.sleep(1)

    rb.close()

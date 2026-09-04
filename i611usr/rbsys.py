# -*- coding: utf-8 -*-
u"""Interface modules to communicate with the control manager（システムマネージャと通信するモジュール）

**Class Hierarchy Chart （クラス階層図）**

.. image:: rbsys.png
"""

import json
import socket,os
import threading
import i611_common
from errlog import liberr
import i611_MCS

#[Internal] --------------------------
def version_rbsys():
    u"""Private （非公開）"""
    return [0, 3, 3, 0]

##======================================================================
##[Public Class]  RobSys: システムマネージャと通信するためのクラス
##======================================================================
class RobSys(object):
    u"""Interface to communicate with the control manager（システムマネージャと通信する）"""

    #[Internal] --------------------------
    def __init__(self, host='127.0.0.1'):
        u"""Instantiates a communication class. （コンストラクタ。ロボットシステムのインスタンスを作成する）

        Args:
            host(str): IP address of robot controller （接続先のIP アドレス指定）

                * Default （初期値）: '127.0.0.1'

        Return:
            RobSys: Reference to instance （RobSysクラスオブジェクトへの参照）

        **Example**::

            # Ex1: omit parameter  （例1：引数省略( 初期値を使用する)）
            rbs = RobSys()

            # Ex2: specify host （例2：ホスト指定）
            rbs = RobSys( host='127.1.1.1' )

        """
        if i611_MCS.has_i611Robot_instance():
            liberr(4,38)
        self.__host = host
        self.__port = 4000
        self.__sock = None
        self.__lock = threading.Lock()
        self.__sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__is_open = False

    #[Internal] --------------------------
    def __del__(self):
        u"""Destructor （デストラクタ）"""
        if not i611_MCS.has_i611Robot_instance():
            self.close()

    #[Public] ############################
    def open(self):
        u"""Starts communication with the control manager. （システムマネージャとの通信を開始する）

        Args:
            None

        Return:
            None

        **Example**::

            rbs.open()

        """
        if not self.__is_open:
            self.__sock.connect((self.__host, self.__port))
            self.__is_open = True

    #[Public] ############################
    def close(self):
        u"""Ends communication with the control manager. （システムマネージャとの接続を終了する）

        Args:
            None

        Return:
            None

        **Example**::

            rbs.close()

        """
        if self.__is_open:
            self.__sock.close()
            self.__is_open = False

    #[Internal] --------------------------
    @staticmethod
    def chkRes(cmdid, buf):
        u"""Private （非公開）"""
        jsonobj = json.loads(buf)
        if cmdid != jsonobj['cmd']:
            return [False, cmdid, 99]
        return jsonobj['results']


    #[Public] ############################
    def set_robtask(self, fname):
        u"""Registers a robot task name. （ロボットプログラムを登録する）

        Args:
            fname(str): Program file name （プログラムファイル名）

        Return:
            bool: Command result （登録処理の実行結果）

                * True: Success （成功）
                * False: Failure [program not found] （失敗[指定されたファイルが存在しない]）

        **Example**::

            rbs.set_robtask( 'sample01.py' )

        """
        params = {'cmd':1, 'params':[fname]}
        with self.__lock:
            if self.__sock.send(json.dumps(params)) <= 0:
                return liberr(4,19)
            buf = self.__sock.recv(256)
        return self.chkRes(params['cmd'], buf)


    #[Public] ############################
    def get_robtask(self):
        u"""Obtains the state of the robot task. （ロボットプログラムの状態を取得する）

        Args:
            None

        Return:
            list: [res0, res1, res2]
                res0(bool): Status of robot program （ロボットプログラムの状態）

                    * True: Running. （ロボットプログラム実行中）
                    * False: Stopped （停止中）

                res1(int): Process ID of robot program （実行中または最後に実行されたロボットプログラムのプロセスID）
                res2(str): File name of registered program （登録されているロボットプログラムファイル名）

        **Example**::

            # Get robot status（ロボットプログラムの実行状態を取得する）
            rb.get_robtask()[0]

            # Get process id of robot program （実行中または最後に実行されたロボットプログラムのプロセスID を取得する）
            rb.get_robtask()[1]

            # Get registered program name （登録されているロボットプログラムファイル名を取得する）
            rb.get_robtask()[2]

        """
        params = {'cmd':2, 'params':[]}
        with self.__lock:
            if self.__sock.send(json.dumps(params)) <= 0:
                return liberr(4,19)
            buf = self.__sock.recv(256)
        return self.chkRes(params['cmd'], buf)


    #[Public] ############################
    def clear_robtask(self):
        u"""Unregisters the robot task.（ロボットプログラムの登録を解除する）

        Args:
            None

        Return:
            bool: Command result

                * True: Success （成功）
                * False: Failure （失敗）

        **Example**::

            # Success （成功）
            if rbs.clear_robtask()[0] == True:
                ...

            # Failure （失敗）
            if rbs.clear_robtask()[0] == False:
                ...

        .. hint:
            * This method doesn't stop the running program.
            （clear_robtask() メソッドでは実行中のロボットプログラムは停止しません。）

        """
        params = {'cmd':3, 'params':[]}
        with self.__lock:
            if self.__sock.send(json.dumps(params)) <= 0:
                return liberr(4,19)
            buf = self.__sock.recv(256)
        return self.chkRes(params['cmd'], buf)


    #[Public] ############################
    def req_mcmd(self):
        u"""[Depricated]Inquires and notifies the state of the robot motion command.（システム状態および動作コマンド状態を取得する）

            Use i611Robot.get_system_port() instead.

        Args:
            None

        Return:
            list: [running, svon, emo, hw_error, sw_error, abs_lost, in_pause, error]

                * running: True when program is running （ロボットプログラム状態）
                * svon: True when servo power turn on （サーボ状態）
                * emo: True when emergency button is active （非常停止状態）
                * hw_error: True when hardware error occurs （システム定義エラー( 致命的) 状態）
                * sw_error: True when software error occurs （システム定義エラー状態）
                * abs_lost: True when ABS has lost （ABS 消失状態）
                * in_pause: True when program is paused （一時停止状態）
                * error: True when any error occurs （システムエラー状態）

        **Example**::

            result = rbs.req_mcmd()
            running = result[0]
            svon = result[1]
            emo = result[2]
            hw_error = result[3]
            sw_error = result[4]
            abs_lost = result[5]
            in_pause = result[6]
            error = result[7]


        """
        params = {'cmd':4, 'params':[]}
        with self.__lock:
            if self.__sock.send(json.dumps(params)) <= 0:
                return liberr(4,19)
            buf = self.__sock.recv(256)
        return self.chkRes(params['cmd'], buf)


    #[Public] ############################
    def cmd_run(self, fname=None):
        u"""Command: Run/Resume（動作コマンド：ロボットプログラムを実行する）

        Resumes program when it is paused. （一時停止中であれば、プログラムの実行を再開します）

        Args:
            fname(str): ファイル名

                * Default: None(set_robtask() で指定したロボットプログラムが実行されます。)

        Return:
            list: [result]

                * result(bool): Command Result（実行結果）

                    - True: 成功
                    - False: 失敗

        **Example**::

            # Ex1: run program registered by set_robtask()
            # 例1：set_robtask() で指定したロボットプログラムを実行する。引数は省略する。
            rbs.set_robtask('sample.py')
            rbs.cmd_run()

            # Ex2: run program specified as parameter
            # 例2：引数でファイル名を指定する
            rbs.cmd_run('sample.py')

        """
        if fname!=None:
            if os.path.exists(fname):
                fname = os.path.abspath(fname)
            else:
                return liberr(4,32,"RobSys.cmd_run")
        params = {'cmd':8, 'params':[fname]}
        with self.__lock:
            if self.__sock.send(json.dumps(params)) <= 0:
                return liberr(4,19)
            buf = self.__sock.recv(256)
        return self.chkRes(params['cmd'], buf)


    #[Public] ############################
    def cmd_stop(self):
        u"""Command: Abort（動作コマンド：減速停止する）

        Args:
            None

        Return:
            list: [result]

                * result(bool): Command Result（実行結果）

                    - True: 成功
                    - False: 失敗

        **Examples**::

            rbs.cmd_stop()

        .. hint::
            **Relation cmd_stop() and i611RObt.enable_interrupt() （cmd_stop() とi611Robot クラスのenable_interrupt() の関係）**

            Behavior of cmd_stop is supposed to be changed according to enable_interrupt() setting. （i611Robot クラスのenable_interrupt()（減速停止割り込み設定）により、減速停止後の挙動が異なります。）

                +--------------------------------+------------------------------------+
                |Stop interrupt(減速停止割り込み)|Behavior after stopped (減速停止後) |
                |enable_interrupt(eid, enable)   |                                    |
                +================================+====================================+
                |Enabled (有効)                  |Exception raised. (例外が発生します)|
                |enable_interrupt( 0, True )     |                                    |
                +--------------------------------+------------------------------------+
                |Disabled (無効)                 |Finish program.  (正常に終了します) |
                |enable_interrupt( 0, False )    |                                    |
                +--------------------------------+------------------------------------+

            If stop interrupt exception is not catched, program will finished by exception.
            （減速停止の割り込みが有効な状態で、ロボットプログラムで例外が処理されなかった場合は、ロボットプログラムは異常終了します。）

        """
        params = {'cmd':13, 'params':[]}
        with self.__lock:
            if self.__sock.send(json.dumps(params)) <= 0:
                return liberr(4,19)
            buf = self.__sock.recv(256)
        return self.chkRes(params['cmd'], buf)


    #[Public] ############################
    def cmd_reset(self):
        u"""Command: Reset error（動作コマンド：エラーリセットする）

        Args:
            None

        Return:
            list: [result]

                * result(bool): Command Result（実行結果）

                    - True: 成功
                    - False: 失敗


        **Example**::

            rbs.cmd_reset()

        .. hint::
            Error list that can be reset by cmd_reset()（cmd_reset()でリセットできるエラー）

                +--------------------------------------------------------+------------------------------+
                |Error type                                              |can be reset（リセットできる）|
                +===+====================================================+==============================+
                |E**|System error（システム定義エラー）                  |Yes                           |
                +---+----------------------------------------------------+------------------------------+
                |c**|System fatal error（システム定義エラー[致命的]）    |No                            |
                +---+----------------------------------------------------+------------------------------+
                |u**|User-defined error（ユーザ定義エラー）              |Yes                           |
                +---+----------------------------------------------------+------------------------------+
                |r**|User-defined fatal error（ユーザ定義エラー[致命的]）|No                            |
                +---+----------------------------------------------------+------------------------------+


        """
        params = {'cmd':14, 'params':[]}
        with self.__lock:
            if self.__sock.send(json.dumps(params)) <= 0:
                return liberr(4,19)
            buf = self.__sock.recv(256)
        return self.chkRes(params['cmd'], buf)


    #[Public] ############################
    def cmd_pause(self):
        u"""Command: Pause（動作コマンド：一時停止する）

        Args:
            None

        Return:
            list: [result]

                * result(bool): Command Result（実行結果）

                    - True: 成功
                    - False: 失敗


        **使用例**::

            rbs.cmd_pause()

        .. hint::
            About effective timing of cmd_pause() （cmd_pause() のタイミングについて）

                cmd_pause() command is effective when program is executing in any move command or user_hook() command.
                cmd_run() command can resume paused program.
                （一時停止は動作コマンド内およびuser_hook() メソッドを実行するタイミングで、cmd_pause()
                が実行されていれば一時止することができます。動作を再開するにはcmd_run() で行います。）

        """
        params = {'cmd':21, 'params':[]}
        with self.__lock:
            if self.__sock.send(json.dumps(params)) <= 0:
                return liberr(4,19)
            buf = self.__sock.recv(256)
        return self.chkRes(params['cmd'], buf)


    #[Public] ############################
    # Order G3[ 0:run,       1:stop,     2:err_reset,    3:pause     ]
    def assign_din(self, *arg1, **arg2):
        u"""Assigns functions to the physical I/O or memory I/O inputs specified.（物理I/O とメモリI/O の入力ポートに機能を割り付ける）

        Args:
            run(int): Input port number for run command（ロボットプログラム実行に割り当てる入力ポート番号）

                * Default（初期値）: -1
            stop(int): Input port number for stop command （減速停止に割り当てる入力ポート番号）

                * Default（初期値）: -1
            err_reset(int): Input port number for reset command （エラーリセットに割り当てる入力ポート番号）

                * Default（初期値）: -1
            pause(int): Input port number for pause command （一時停止に割り当てる入力ポート番号）

                * Default（初期値）: -1

        Return:
            bool: True when assignment success. （成功した場合にTrueを返します。失敗したときは例外が発生します。）

        Note:
            Port number range （引数の設定範囲）:
                * Physical input port （物理I/O）: 0 - 15
                * Memory I/O port （メモリI/O）: 32 - 12287

            * Specify -1 not to assign port number. （割り付けない場合は-1 を指定します。）
            * Omit parameter to set default. （引数を省略すると初期値が設定されます。）
            * Cannot assign different command to the same port. （同一のI/O に重複して割り付けることはできません。）
            * Physical input ports are relating to IN1 - IN16 in CN3. （設定範囲0 - 15 は入力ポート信号名のIN1 ～ IN16（CN3：I/O コネクタ1）に相当します。）
            * Input signal detects with the rising edge. （入力信号はLow → Hi の入力エッジを検出します。）

        **Example**::

            # Recommend to specify as below in init.py （init.py で指定します：（以下は推奨設定））
            rbs.assign_din( run=0, stop=1, err_reset=2, pause=3 )

        """
        p = i611_common._args([-1, -1, -1, -1],
              ['run', 'stop', 'err_reset', 'pause'],
              [int, int, int, int], *arg1, **arg2)
        if p[0] == False:
            return liberr(4,2)

        params = {'cmd':17, 'params':[p[1], p[2], p[3], p[4]]}
        with self.__lock:
            if self.__sock.send(json.dumps(params)) <= 0:
                return liberr(4,19)
            buf = self.__sock.recv(256)
        return self.chkRes(params['cmd'], buf)


    #[Public] ############################
    # Order [ 0:svon,      1:emo,      2:hw_error,     3:abs_lost  ]
    #       [ 0:running,   1:sw_error, 2:in_pause,     3:error     ]
    def assign_dout(self, *arg1, **arg2):
        u"""Assigns functions to the physical I/O or memory I/O output specified.（物理I/O とメモリI/O の出力ポートに機能を割り付ける）

        Args:
            running(int): Output port number for running status（ロボットプログラム状態に割り当てる出力ポート番号）

                * Default （初期値）: -1
            svon(int): Output port number for servo power status （サーボ状態に割り当てる出力ポート番号）

                * Default （初期値）: -1
            emo(int): Output port number for emergency status （非常停止状態に割り当てる出力ポート番号）

                * Default （初期値）: -1
            hw_error(int): Output port number for hardware error （システム定義エラー( 致命的) 状態に割り当てる出力ポート番号）

                * Default （初期値）: -1
            sw_error(int): Output port number for software error （システム定義エラー状態に割り当てる出力ポート番号）

                * Default （初期値）: -1
            abs_lost(int): Output port number for ABS lost error （ABS 消失状態に割り当てる出力ポート番号）

                * Default （初期値）: -1
            in_pause(int): Output port number for pause status （一時停止状態に割り当てる出力ポート番号）

                * Default （初期値）: -1
            error(int): Output port number for any error(*) （システムエラー状態に割り当てる出力ポート番号 (*)）

                * Default （初期値）: -1

        Return:
            bool: True when assignment success. （成功した場合にTrueを返します。失敗したときは例外が発生します。）

        Note:
            Port number range （設定範囲）:
                * Physical output port （物理I/O）: 16 - 31
                * Memory I/O port （メモリI/O）: 32 - 12287

            * Specify -1 not to assign port number. （割り付けない場合は-1 を指定します。）
            * Omit parameter to set default. （引数を省略すると初期値が設定されます。）
            * Cannot assign different command to the same port. （同一のI/O に重複して割り付けることはできません。）
            * Physical output ports are relating to O1 - O16 in CN4 nad CN5. （設定範囲16 - 31 は信号名のO1 ～ O16（CN4：I/O コネクタ2、CN5：I/O コネクタ3）に相当します。）
            * When output is On, relating port becomes Hi. （出力がON になると、対応する出力ポートがHi になります。）

        See Also:
            (*) True when either system error or system fatal error occurs.（Sysシステム定義エラー（非致命的または致命的）の発生状態です。２つのエラー状態を1本の制御線で確認する場合に使用します。）

        **Example**::

            # Recommend to specify as below in init.py （init.py で指定します：（以下は推奨設定））
            rbs.assign_dout( running=16, svon=17, emo=18, hw_error=19, sw_error=20,
            abs_lost=21, in_pause=22, error=23 )

        """
        p = i611_common._args([-1, -1, -1, -1, -1, -1, -1, -1],
            ['running', 'svon', 'emo', 'hw_error','sw_error', 'abs_lost', 'in_pause', 'error'],
            [int, int, int, int, int, int, int, int],
            *arg1, **arg2)

        if p[0] == False:
            return liberr(4,2)

        params = {'cmd':18, 'params':[p[2], p[3], p[4], p[6], p[1], p[5], p[7], p[8]]}
        with self.__lock:
            if self.__sock.send(json.dumps(params)) <= 0:
                return liberr(4,19)
            buf = self.__sock.recv(256)
        return self.chkRes(params['cmd'], buf)


    #[Public] ############################
    def version(self):
        u"""Get the version of system manager. （システムマネージャのバージョンを取得する）

        Args:
            None

        Return:
            list: [res0, major, minor, patch, build]

                * res0: True:Success（成功）、False:Failure（失敗）
                * major: Major version（メジャーバージョン）
                * minor: Minor version（マイナーバージョン）
                * patch: Patch version（パッチバージョン）
                * build: Build version （ビルドバージョン）

        **Example**::
            version=rbs.version()
            print version

        """
        params = {'cmd':20, 'params':[]}
        with self.__lock:
            if self.__sock.send(json.dumps(params)) <= 0:
                return liberr(4,19)
            buf = self.__sock.recv(256)
        return self.chkRes(params['cmd'], buf)


    #[Internal] --------------------------
    def register_program(self,pid):
        u"""(非公開)"""
        params = {'cmd':22, 'params':[pid]}
        with self.__lock:
            if self.__sock.send(json.dumps(params)) <= 0:
                return liberr(4,19)
            buf = self.__sock.recv(256)
        return self.chkRes(params['cmd'], buf)


    #[Internal] --------------------------
    def debug_cmd(self,cmd):
        u"""(非公開)"""
        params = {'cmd':23, 'params':[cmd]}
        with self.__lock:
            if self.__sock.send(json.dumps(params)) <= 0:
                return liberr(4,19)
            buf = self.__sock.recv(256)
        return self.chkRes(params['cmd'], buf)


    #[Internal] --------------------------
    def _internal_call(self,cmd):
        u"""(非公開)"""
        params = {'cmd':24, 'params':cmd}
        with self.__lock:
            if self.__sock.send(json.dumps(params)) <= 0:
                return liberr(4,19)
            buf = self.__sock.recv(256)
        return self.chkRes(params['cmd'], buf)


#[EOF]

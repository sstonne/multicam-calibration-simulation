#!/usr/bin/env python
# -*- coding: utf-8 -*-


import time
import sys
import math

from i611shm import shm_read

class Cui(object):
    CHR_HOME  =   "\033[0;0H"
    CHR_CLEAR =   "\033[2J"
    CHR_RESET =   "\x1b[0m"
    CHR_DELLINE = "\033[K"
    CHR_BEEP  =   "\x07"
    # ID for col()
    BLACK, RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, WHITE = range(8)
    COL_BOLD = ("\x1b[1;30m", "\x1b[1;31m", "\x1b[1;32m", "\x1b[1;33m",
           "\x1b[1;34m", "\x1b[1;35m", "\x1b[1;36m", "\x1b[1;37m")
    COL = ("\x1b[30m", "\x1b[31m", "\x1b[32m", "\x1b[33m",
           "\x1b[34m", "\x1b[35m", "\x1b[36m", "\x1b[37m")
    enb = True
    @classmethod
    def enable(cls,sw):
        cls.enb = sw
    @classmethod
    def clear(cls):
        if cls.enb:
            print cls.CHR_CLEAR,
    @classmethod
    def home(cls):
        if cls.enb:
            print cls.CHR_HOME,
    @classmethod
    def beep(cls):
        if cls.enb:
            print cls.CHR_BEEP,
    @classmethod
    def delline(cls):
        if cls.enb:
            print cls.CHR_DELLINE,
    @classmethod
    def col(cls, msg, col_id=RED, bold=True):
        msg = str(msg)
        if cls.enb:
            if bold:
                return "%s%s%s" % (cls.COL_BOLD[col_id], msg, cls.CHR_RESET)
            else:
                return "%s%s%s" % (cls.COL[col_id], msg, cls.CHR_RESET)
        else:
            return msg

def read_position_info():
    info = shm_read(0x3000,20).split(',')
#    print info
    pos = []
    pos.append(float(info[0])*1000)
    pos.append(float(info[1])*1000)
    pos.append(float(info[2])*1000)
    pos.append(math.degrees(float(info[3])))
    pos.append(math.degrees(float(info[4])))
    pos.append(math.degrees(float(info[5])))
    pos.append(int(info[6]))
    pos.append(int(info[8]))
    jnt = []
    jnt.append(math.degrees(float(info[9])))
    jnt.append(math.degrees(float(info[10])))
    jnt.append(math.degrees(float(info[11])))
    jnt.append(math.degrees(float(info[12])))
    jnt.append(math.degrees(float(info[13])))
    jnt.append(math.degrees(float(info[14])))
    verocity = float(info[17])            # verocity
    singular = int(info[7])             # singular
    softlimit = int(info[19])           # softlimit

    return pos, jnt, verocity,singular, softlimit

def read_tcp_info():
    info = shm_read(0x30E8,6).split(',')
#    print info
    pos = []
    pos.append(float(info[0])*1000)
    pos.append(float(info[1])*1000)
    pos.append(float(info[2])*1000)
    pos.append(math.degrees(float(info[3])))
    pos.append(math.degrees(float(info[4])))
    pos.append(math.degrees(float(info[5])))
    pos.append(0)
    pos.append(0)
    return pos

def pos2str(pos,header="Pos"):
    msg = "%s,  %7.2f,%7.2f,%7.2f,%7.2f,%7.2f,%7.2f, %d, %08X   " % (
        header,pos[0],pos[1],pos[2],pos[3],pos[4],pos[5],pos[6],pos[7])
    return msg

def jnt2str(jnt):
    msg = "Jnt,  %7.2f,%7.2f,%7.2f,%7.2f,%7.2f,%7.2f   " % (
        jnt[0],jnt[1],jnt[2],jnt[3],jnt[4],jnt[5])
    return msg

def softlimit2str(softlimit):
    msg = "Limit,"
    for i in range(6):
        st =  (softlimit >> (20 - 4*i)) & 0x0F
        if st == 2:
            stmsg = Cui.col(" Upper",Cui.RED)
        elif st == 1:
            stmsg = Cui.col(" Upper",Cui.YELLOW)
        elif st == 0:
            stmsg = "Normal"
        elif st == 0x0F:
            stmsg = Cui.col(" Lower",Cui.YELLOW)
        elif st == 0x0E:
            stmsg = Cui.col(" Lower",Cui.RED)
        else:
            stmsg = Cui.col("?(%3d)" % st,Cui.RED)
        msg += " " + stmsg
        if i<5:
            msg += ","
    return msg

def status2str(st):
    if st == 0:
        return Cui.col("0", Cui.GREEN)
    else:
        return Cui.col("1", Cui.RED)


def main():
    single_shot = False
    if len(sys.argv)>1:
        if sys.argv[1] == '-s':
            single_shot = True
            Cui.enable(False)

    Cui.home()
    Cui.clear()

    while True:
        Cui.home()
        pos, jnt, verocity, singular, softlimit = read_position_info()
        _ = verocity # not used
        tcp = read_tcp_info()

        print  Cui.col("        X/J1,   Y/J2,   Z/J3,  Rz/J4,  Ry/J5,  Rx/J6, P, Mt",Cui.MAGENTA, False)

        pos_str = pos2str(pos)
        tcp_str = pos2str(tcp,"TCP")
        jnt_str = jnt2str(jnt)
        softlimit_str = softlimit2str(softlimit)
        print Cui.col(pos_str,Cui.BLUE)
        print Cui.col(tcp_str,Cui.BLUE)
        print Cui.col(jnt_str,Cui.GREEN)
        print softlimit_str
        print
        print "Singular Status:"
        print "  [%s] Right/Left" % status2str(singular & (0x01<<0))
        print "  [%s] Upper/Lower elbow" % status2str(singular & (0x01<<1))
        print "  [%s] Wrist flip/non flip" % status2str(singular & (0x01<<2))
        print "  [%s] soft limit" % status2str(singular & (0x01<<16))
        print "  [%s] unreachable point" % status2str(singular & (0x01<<17))

        if single_shot:
            break
        time.sleep(0.5)

if __name__ == "__main__":
    main()
#eof

#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
release_permission.py
컨트롤러 권한 반납 + 에러 상태 클리어

사용법: python release_permission.py
"""
import sys
import atexit
import os
import time
import i611_MCS
from i611_MCS import i611Robot
from rbsys import RobSys

# ── Step 1: RobSys로 에러 상태 클리어 ────────────────────────
# i611Robot() 생성 전에 해야 함 (생성 시 st>=10 이면 바로 죽음)
print "[1] RobSys 에러 리셋 시도..."
try:
    rbs = RobSys()
    res = rbs.cmd_reset()
    print "    cmd_reset result: %s" % str(res)
except Exception as e:
    print "    cmd_reset error: %s" % str(e)

time.sleep(0.5)

# ── Step 2: i611Robot으로 권한 반납 ──────────────────────────
print "[2] permission 반납 시도..."
try:
    rb = i611Robot()
    rb.open(permission=False)

    # atexit 훅 제거 (소켓 닫힌 후 컨트롤러 에러 상태 만드는 원인)
    try:
        atexit._exithandlers = [
            (f, args, kwargs)
            for (f, args, kwargs) in atexit._exithandlers
            if f != rb._hook_atexit
        ]
        print "    atexit hook removed"
    except Exception as e:
        print "    atexit hook remove error: %s" % str(e)

    # sys 훅 복원
    try:
        sys.excepthook = rb._org_excepthook
        sys.exit       = rb._org_exit
    except AttributeError:
        pass

    rb._rel_permission()
    print "    permission released"

    rb._close()
    i611_MCS.ONCE_OPENED = False

except Exception as e:
    print "    error: %s" % str(e)

print "[done] 이제 move.py 를 실행하세요."
os._exit(0)

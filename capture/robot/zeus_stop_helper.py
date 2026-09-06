#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Separate process: RobSys cannot coexist with an i611Robot instance.

probe connects and reads status only; stop requests controller deceleration stop.
Neither operation resets errors, switches servo power, or starts a program.
"""
from __future__ import print_function
import argparse
import json
import os
import socket
import sys


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('action', choices=['probe', 'stop'])
    args = ap.parse_args()
    socket.setdefaulttimeout(2.0)
    if os.environ.get('ZEUS_CAPTURE_SDK_DIR'):
        sys.path.insert(0,os.environ['ZEUS_CAPTURE_SDK_DIR'])
    from rbsys import RobSys
    for name in ('open','close','req_mcmd','cmd_stop'):
        if not callable(getattr(RobSys,name,None)):
            raise RuntimeError('Required stop transport API unavailable: '+name)
    rbs = RobSys()
    try:
        rbs.open()
        result = rbs.req_mcmd() if args.action == 'probe' else rbs.cmd_stop()
        if args.action == 'stop' and (not isinstance(result,(list,tuple)) or
                                     not result or result[0] is not True):
            raise RuntimeError('Stop request rejected: %r' % (result,))
        if args.action == 'probe' and (not isinstance(result,(list,tuple)) or len(result)<8):
            raise RuntimeError('Unexpected system status: %r' % (result,))
        print(json.dumps({'action':args.action, 'result':result}))
    finally:
        rbs.close()


if __name__ == '__main__':
    main()

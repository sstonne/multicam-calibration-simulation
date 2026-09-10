#!/usr/bin/python
# -*- coding: utf-8 -*-
"""Legacy entry point. Deploy pose_server.py in the same directory.

Both names now serve read-only shared-memory poses on port 12352.
"""
from pose_server import main


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nStopped')

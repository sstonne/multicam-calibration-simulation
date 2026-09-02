# -*- coding: utf-8 -*-
from rbsys import *

#This is sample program for initial settings.

if __name__ == "__main__":
  rbs = RobSys()
  rbs.open()

  rbs.assign_din(run=0)
  rbs.set_robtask("running.py")

  rbs.close()

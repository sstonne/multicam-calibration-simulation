# -*- coding: utf-8 -*-
#!/usr/bin/python

from i611_MCS import *
from teachdata import *
from i611_extend import *
from rbsys import *
from i611_common import *
from i611_io import *
from i611shm import *
import sys
import json
import time

from FACETRACKING import FaceTrackingFeverServer
from MEDICINE4 import MedicineHandoverServer
from server_comm import RobotEnvServer

def main():
    print('main start')
    # 열 측정 모듈 실행 (로봇 객체 공유)
    fever = FaceTrackingFeverServer(rb, server)
    print "[SERVER] fever_facetracking ing...."
    fever.run()

    # 약 전달 모듈 실행 (로봇 객체 공유)
   # med = MedicineHandoverServer(rb, server)
    #print "[SERVER] medicineHandober ing..."
   # med.run()


if __name__ == "__main__":
    try:
	rb = i611Robot()
	_BASE = Base()
	rb.open()
	IOinit(rb)
        rb.settool(1, 0.0, 0.0, 245.0, 0.0, 0.0, 0.0)
        rb.changetool(1)
        rb.settool(2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

	server = RobotEnvServer()
        server.start()
	print(server)
        print("[SERVER] Waiting for client ready..")
	
        main()

        server.close()
    except KeyboardInterrupt:
	print('KeyboardInterrupt')
	rb.exit(0)
	rb.close()
    except Exception, e:
	print('error: ', e.__class__.__name__, ':', e)
	rb.exit(0)
	rb.close(0)

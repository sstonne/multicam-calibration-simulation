# -*- coding: utf-8 -*-
#!/usr/bin/python

from i611_MCS import *
from face_tracking_fever_servertong import FaceTrackingFeverServer
from mediserver_tong import MedicineHandoverServer

def main():
    rb=None
    try:
        # 로봇 한 번만 초기화
        rb = i611Robot()
        rb.open()
        rb.settool(1, 0.0, 0.0, 245.0, 0.0, 0.0, 0.0)
        rb.changetool(1)
        rb.settool(2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


        # 열 측정 모듈 실행 (로봇 객체 공유)
        fever = FaceTrackingFeverServer(rb)
	print "[SERVER] fever_facetracking ing...."
        fever.run()

        # 약 전달 모듈 실행 (로봇 객체 공유)
        med = MedicineHandoverServer(rb)
        print "[SERVER] medicineHandober ing..."
	med.run()

    except KeyboardInterrupt:
        print "[SERVER] Running MedicineHandoverServer..."
    except Exception, e:
        print "[SERVER] Error:", e
    finally:
        if rb:
            try:
                rb.close()
            except:
                pass
                    
        print "[SYSTEM] !!!!DONE!!!!"

if __name__ == "__main__":
    main()


# -*- coding: utf-8 -*-

import time
from i611_io import *

IOinit()

patterns = [
    ("ALL OFF", "0000"),
    ("OUT 0001", "0001"),
    ("OUT 0010", "0010"),
    ("OUT 0100", "0100"),
    ("OUT 1000", "1000"),
    ("ALL ON",  "1111"),
]

for name, p in patterns:
    print("")
    print("================================")
    print(name, p)
    print("10초 동안 유지합니다.")
    print("그리퍼 / 솔레노이드 LED / 클릭음 확인")
    print("================================")

    dout(48, "0000")
    time.sleep(1)

    dout(48, p)
    time.sleep(10)

dout(48, "0000")
print("DONE")

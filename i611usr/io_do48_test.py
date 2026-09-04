# -*- coding: utf-8 -*-

import time
from i611_io import *

IOinit()

try:
    input_func = raw_input
except NameError:
    input_func = input

patterns = [
    ("ALL OFF", "0000"),
    ("DO48 ON", "0001"),
    ("DO49 ON", "0010"),
    ("DO50 ON", "0100"),
    ("DO51 ON", "1000"),
    ("ALL ON",  "1111"),
]

for name, pattern in patterns:
    print("")
    print("================================")
    print(name)
    print("dout(48, '{}')".format(pattern))
    print("IO monitor에서 DO48~DO51 변화 확인")
    print("================================")

    dout(48, "0000")
    time.sleep(0.5)

    dout(48, pattern)

    input_func("확인했으면 Enter...")

dout(48, "0000")
print("DONE")

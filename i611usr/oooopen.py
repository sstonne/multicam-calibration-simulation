import time
from i611_io import *

IOinit()

print("RESET OUTPUT")
dout(48, '0000')
time.sleep(1)

print("FORCE OPEN: dout(48, '0100')")
dout(48, '0100')
time.sleep(3)

print("STOP OUTPUT")
dout(48, '0000')

import numpy as np
import time

from pyBlimp.blimp import *

# create serial object
port = "/dev/ttyUSB0"
ser = create_serial(port)

# build the blimp object
b1 = Blimp(1, port, ser, motors_only=True)
b2 = Blimp(2, port, ser, motors_only=True)

while True:
    time.sleep(1)
    # test forward force with blimp 1
    print("Testing forward with blimp 1")
    for t in range(10):
        u = np.zeros(4)
        u[0] = 0.1
        b1.step(u)
        time.sleep(0.1)

    time.sleep(1)
    # test forward force with blimp 2
    print("Testing forward with blimp 2")
    for t in range(10):
        u = np.zeros(4)
        u[0] = 0.1
        b2.step(u)
        time.sleep(0.1)

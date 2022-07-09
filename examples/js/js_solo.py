import matplotlib.pyplot as plt
import numpy as np
import signal
import time
import traceback

from pyBlimp.blimp import Blimp
from pyBlimp.utils import wrap, create_serial
from utils.js_utils import JoyStick_helper

# setup serial device
ser = create_serial("/dev/ttyUSB0")

# build the blimp object
b = Blimp(1, ser, (1111, 1112, 1113))

# setup the joystick reader
js = JoyStick_helper()

# show the FPV
fig, axes = plt.subplots(1,1)

# setup exit on ctrl-c
running = True
def exit_handle(signum, frame):
    global running
    running = False
    
signal.signal(signal.SIGINT, exit_handle)

# desired states to track
des = np.zeros(4)

try:
    while running:
        # handle the joystick
        ax, _, _ = js.get_state()
        
        # decide inputs
        des[0] = np.clip(des[0] - 0.05*ax[3], 0.0, 2.5)
        des[1] = -ax[0]
        des[2] = -ax[1]
        des[3] = wrap(des[3] + 0.05*ax[2])
        b.set_des(des)

        # show the video
        I = b.get_image()
        axes.clear()
        axes.imshow(I)
        axes.set_xticks([])
        axes.set_yticks([])
        axes.set_title("FPV")

        plt.draw(); plt.pause(0.0001)

        # break if the figure is closed
        if not plt.fignum_exists(fig.number): running = False

except:
    traceback.print_exc()
    b.shutdown()

b.shutdown()

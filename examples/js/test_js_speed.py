import matplotlib.pyplot as plt
import numpy as np
import signal
import time

from pyBlimp.blimp import BlimpManager
from pyBlimp.utils import *
from utils.js_utils import JoyStick_helper

# load desired configs
cfg_paths = ["configs/config1.yaml"]
cfg = read_config(cfg_paths)

# build the blimp object
b = BlimpManager(cfg, "/dev/ttyUSB0", logger=True)

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

started = False

# desired states to track
des = np.zeros(4)
des[3] = 1.5

while running and b.get_running(0):
    # handle the joystick
    ax, start, stop = js.get_state()

    # decide inputs
    des[0] =  0.2*ax[0]
    des[1] =  -0.2*ax[1]
    des[2] = wrap(des[2]-0.05*ax[2])
    des[3] = np.clip(des[3]-0.05*ax[3], 0.0, 2.5)

    if not started and start:
        print("Starting!")
        started = True

    elif started:
        # desired states to track
        des[1] = 0.075
            
    if stop:
        print("Stopping!")
        # fix pitch to estimate a velocity
        started = not stop
        running = False

    b.set_des(des, 0)

    # show the video
    I = b.get_image(0)
    axes.clear()
    axes.imshow(I)
    axes.set_xticks([])
    axes.set_yticks([])
    axes.set_title("FPV")

    plt.draw(); plt.pause(0.0001)

    # break if the figure is closed
    if not plt.fignum_exists(fig.number): running = False

b.shutdown()
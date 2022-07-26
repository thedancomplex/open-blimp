import matplotlib.pyplot as plt
import numpy as np
import signal
import time

from pyBlimp.blimp import BlimpManager
from pyBlimp.utils import *
from utils.js_utils import JoyStick_helper

# setup exit on ctrl-c
running = True
def exit_handle(signum, frame):
    global running
    running = False
    
signal.signal(signal.SIGINT, exit_handle)



# load desired configs
cfg_paths = ["configs/config1.yaml", "configs/config2.yaml"]
cfg = read_config(cfg_paths)

# setup the joystick reader
js1 = JoyStick_helper(js_id=0)
js2 = JoyStick_helper(js_id=1)

# build the blimp object
b = BlimpManager(cfg, "/dev/ttyUSB0", logger=True)

# show the FPVs
fig, axes = plt.subplots(1,2)


# desired states to track
des1 = np.zeros(4)
des1[3] = 1.5

des2 = np.zeros(4)
des2[3] = 1.5


while running and b.get_running(0):
    # handle the joystick
    ax1, _, _ = js1.get_state()
    ax2, _, _ = js2.get_state()
 
    # decide inputs
    des1[0] =  0.1*ax1[0]
    des1[1] =  0.1*ax1[1]
    des1[2] = wrap(des1[2]-0.05*ax1[2])
    des1[3] = np.clip(des1[3]+0.05*ax1[3], 0.0, 2.5)
    b.set_des(des1, 0)

    des2[0] =  0.1*ax2[0]
    des2[1] =  0.1*ax2[1]
    des2[2] = wrap(des2[2]-0.05*ax2[2])
    des2[3] = np.clip(des2[3]+0.05*ax2[3], 0.0, 2.5)
    b.set_des(des2, 1)

    # show the video
    I1 = b.get_image(0)
    I2 = b.get_image(1)

    axes[0].clear()
    axes[0].imshow(I1)
    axes[0].set_xticks([])
    axes[0].set_yticks([])
    axes[0].set_title("FPV 1")

    axes[1].clear()
    axes[1].imshow(I2)
    axes[1].set_xticks([])
    axes[1].set_yticks([])
    axes[1].set_title("FPV 2")

    plt.draw(); plt.pause(0.0001)

    # break if the figure is closed
    if not plt.fignum_exists(fig.number): running = False

b.shutdown()

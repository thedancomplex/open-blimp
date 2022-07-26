import matplotlib.pyplot as plt
import numpy as np
import signal
import time

from pyBlimp.blimp import BlimpManager
from pyBlimp.utils import *
from utils.js_utils import JoyStick_helper
from vicon.utils import *
from vicon.viconReader import ViconInterface

# setup exit on ctrl-c
running = True
def exit_handle(signum, frame):
    global running
    running = False
    
signal.signal(signal.SIGINT, exit_handle)


# build the Vicon interface
names = parse_names("vicon/names.txt")
V = ViconInterface()

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

# synchronize signal
b.set_extra(names[-1], True, 0)
b.set_extra(names[-1], True, 1)

print("Starting!")
while running and b.get_running(0):
    # handle the joystick
    ax1, _, _ = js1.get_state()
    ax2, _, _ = js2.get_state()
 
    # decide inputs
    des1[0] =  0.5*ax1[0]
    des1[1] =  -0.5*ax1[1]
    des1[2] = wrap(des1[2]-0.05*ax1[2])
    #b.set_des(des1, 0)

    des2[0] =  0.5*ax2[0]
    des2[1] =  -0.5*ax2[1]
    des2[2] = wrap(des2[2]-0.05*ax2[2])
    #b.set_des(des2, 1)

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

    # store the extra data
    for name in names[:-1]:
        p = V.get_pose(name)
        b.set_extra(name, p[1:], 0)
        b.set_extra(name, p[1:], 1)

    # break if the figure is closed
    if not plt.fignum_exists(fig.number): running = False

b.set_extra(names[-1], False, 0)
b.set_extra(names[-1], False, 1)

b.shutdown()
V.shutdown()

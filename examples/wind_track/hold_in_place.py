import matplotlib.pyplot as plt
import numpy as np
import signal
import time

from pyBlimp.blimp import BlimpManager
from pyBlimp.utils import *
from utils.at_utils import *
from utils.js_utils import JoyStick_helper
from vicon.utils import *
from vicon.viconReader import ViconInterface
from utils.wind import *

# setup exit on ctrl-c
running = True
def exit_handle(signum, frame):
    global running
    running = False
    
signal.signal(signal.SIGINT, exit_handle)


# load desired configs
cfg_paths = ["configs/config1.yaml"]
cfg = read_config(cfg_paths)

# build the blimp object
b = BlimpManager(cfg, "/dev/ttyUSB0", logger=True)

# setup the joystick reader
js = JoyStick_helper()

# build the apriltag tracker
at = AprilTagHelper("utils/K_360x240.txt")

# build the Vicon interface
names = parse_names("vicon/names.txt")
V = ViconInterface()

# show the FPV
fig, axes = plt.subplots(1,1)

# desired states to track
des = np.zeros(4)
des[3] = 1.5

# setup wind-field params
centers = 0.5*np.array([[-1,1,1,-1],[-1,-1,1,1]])
centers[0,:] += 4.

k = 0.5
wHat = None
px = np.zeros(2)
px_ = np.zeros(2)
first = True
while running and b.get_running(0):
    # handle the joystick
    ax, trigger, _ = js.get_state()

    I = b.get_image(0)
    tags, I_ = at.detect(I, label=True)

    # decide inputs with wind-cancelling
    des[0] =  0.2*ax[0]
    des[1] =  -0.2*ax[1]
    des[2] = wrap(des[2]-0.05*ax[2])
    des[3] = np.clip(des[3]-0.05*ax[3], 0.0, 2.5)

    if len(tags) > 0:
        if first:
            px[0] = tags[0].pose_t[2]
            px[1] = tags[0].pose_t[0]
            wHat = windEstimate(px[:,None], centers)
            px_ = px.copy()
            first = False

        else:        
            # get the blimp's position
            px = np.zeros(2)
            px[0] = tags[0].pose_t[2]
            px[1] = tags[0].pose_t[0]

            vW = wHat.predict(px[:,None]); print(vW)
            des[0] += vW[0]
            des[1] += vW[1]

            # compute wind-field update
            dx = px - px_
            px_ = px.copy()
            wHat.step(dx[:,None], des[:2,None])

    b.set_des(des, 0)

    # store the true pose
    for name in names:
        p = V.get_pose(name)
        b.set_extra(name, p[1:], 0)

    # show the video
    axes.clear()
    axes.imshow(I)
    axes.set_xticks([])
    axes.set_yticks([])
    axes.set_title("FPV")

    plt.draw(); plt.pause(0.0001)

    # break if the figure is closed
    if not plt.fignum_exists(fig.number): running = False

b.shutdown()
V.shutdown()

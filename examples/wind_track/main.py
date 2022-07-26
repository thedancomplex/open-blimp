import matplotlib.pyplot as plt
import numpy as np
import signal
import time

from pyBlimp.blimp import BlimpManager
from pyBlimp.utils import *
from utils.at_utils import *


# load desired configs
cfg_paths = ["configs/config1.yaml"]
cfg = read_config(cfg_paths)

# build the blimp object
b = BlimpManager(cfg, "/dev/ttyUSB0")

# build the apriltag tracker
at = AprilTagHelper("utils/K_360x240.txt")

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
des[3] = 1.5

# setup wind-field params
tags = []
while len(tags) == 0:
    I = b.get_image(0)
    tags = at.detect(I)

px = np.zeros(2)
px[0] = tags[0].pose_t[2]
px[1] = -tags[0].pose_t[0]
px_ = px.copy()

centers = 0.5*np.array([[-1,1,1,-1],[-1,-1,1,1]])
centers[0,:] += 4.
wHat = windEstimate(px, centers)

k = 0.5
while running and b.get_running(0):
    # handle the joystick
    ax, trigger, _ = js.get_state()

    I = b.get_image(0)
    tags, I_ = at.detect(I, label=True)

    # get the blimp's position
    px = np.zeros(2)
    px[0] = tags[0].pose_t[2]
    px[1] = -tags[0].pose_t[0]

    # decide inputs as feedback controller
    vW = wHat.predict(px)
    des[0] = -( k*(0-px[1]) - vW[0] )
    des[1] = -( k*(4-px[0]) - vW[1] )
    b.set_des(des, 0)

    # compute wind-field update
    dx = px - px_
    px_ = px.copy()
    wHat.step(dx, des[:2])

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

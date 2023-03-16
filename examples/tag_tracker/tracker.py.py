import matplotlib.pyplot as plt
import numpy as np
import signal
import time

from pyBlimp.blimp import BlimpManager
from pyBlimp.utils import *
from utils.js_utils import JoyStick_helper

if __name__ == "__main__":
    # setup exit on ctrl-c
    running = True
    def exit_handle(signum, frame):
        global running
        running = False
        
    signal.signal(signal.SIGINT, exit_handle)

    # load desired configs
    cfg_paths = ["configs/config2.yaml"]
    cfg = read_config(cfg_paths)

    # build the blimp object
    b = BlimpManager(cfg, "COM3", logger=False)

    # show the FPV
    fig, axes = plt.subplots(1,1)
    I = b.get_image(0)
    h = axes.imshow(I)
    axes.set_xticks([])
    axes.set_yticks([])
    axes.set_title("FPV")

    # desired states to track
    des = np.zeros(4)
    des[3] = 1.5

    while running and b.get_running(0):
        # find the relative position of the apriltag

        # decide inputs
        des[0] =  0.5*ax[0]
        des[1] =  0.5*ax[1]
        des[2] = wrap(des[2]-0.05*ax[2])
        des[3] = np.clip(des[3]+0.05*ax[3], 0.0, 2.5)
        b.set_des(des, 0)

        # show the feed
        I = b.get_image(0)
        h.set_data(I)
        plt.draw(); plt.pause(0.0001)

        # break if the figure is closed
        if not plt.fignum_exists(fig.number): running = False

    b.shutdown()


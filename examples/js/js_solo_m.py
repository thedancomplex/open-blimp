import matplotlib.pyplot as plt
import numpy as np
import signal

from pyBlimp.blimp import Blimp
from pyBlimp.utils import wrap, create_serial
from utils.js_utils import JoyStick_helper


# create serial device and lock pair
ser = create_serial("/dev/ttyUSB1")

# build the blimp object
b = Blimp(ser, id_num=1, ports=(1111, 1112, 1113), logger=False)

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

while running:
    # handle the joystick
    ax, auto_on, _ = js.get_state()

    # decide inputs
    if auto_on:
        des[0] = -0.05*ax[0]
        des[1] =  0.05*ax[1]
        des[2] = wrap(des[2]+0.05*ax[2])
        des[3] = np.clip(des[3]-0.05*ax[3], 0.0, 2.5)
        b.set_des(des)

    else:
        cmd = np.zeros(4)
        cmd[0] = -0.05*ax[1]
        cmd[1] = -0.05*ax[0]
        cmd[2] = -0.05*ax[2]
        cmd[3] = -0.05*ax[3]
        b.set_cmd(cmd)

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

b.shutdown()

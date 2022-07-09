import matplotlib.pyplot as plt
import numpy as np
import signal

from pyBlimp.blimp import Blimp
from pyBlimp.utils import wrap, create_serial
from utils.js_utils import JoyStick_helper


# create serial device and lock pair
ser = create_serial("/dev/ttyUSB1")

# build the blimp object
b1 = Blimp(ser, id_num=1, ports=(1111, 1112, 1113))
b2 = Blimp(ser, id_num=2, ports=(2221, 2222, 2223))

# setup the joystick reader
js = JoyStick_helper()

# show the FPV
fig, axes = plt.subplots(1,2,figsize=(11,5))

# setup exit on ctrl-c
running = True
def exit_handle(signum, frame):
    global running
    running = False
    
signal.signal(signal.SIGINT, exit_handle)

# desired states to track
des1 = np.zeros(4)
des2 = np.zeros(4)
on1, on2 = False, False

while running:
    # handle the joystick
    ax, on1, on2 = js.get_state()
    
    # decide inputs
    if on1:
        des1[0] = -0.05*ax[0]
        des1[1] =  0.05*ax[1]
        des1[2] = wrap(des1[2]+0.05*ax[2])
        des1[3] = np.clip(des1[3]-0.05*ax[3], 0.0, 2.5)
        b1.set_des(des1)

    elif on2:
        des2[0] = -0.05*ax[0]
        des2[1] =  0.05*ax[1]
        des2[2] = wrap(des2[2]+0.05*ax[2])
        des2[3] = np.clip(des2[3]-0.05*ax[3], 0.0, 2.5)
        b2.set_des(des2)

    # show the video feeds
    I1 = b1.get_image()
    I2 = b2.get_image()
    
    axes[0].clear()
    axes[0].imshow(I1)
    axes[0].set_xticks([])
    axes[0].set_yticks([])
    axes[0].set_title("FPV - Blimp 1")
    
    axes[1].clear()
    axes[1].imshow(I2)
    axes[1].set_xticks([])
    axes[1].set_yticks([])
    axes[1].set_title("FPV - Blimp 2")

    plt.draw(); plt.pause(0.0001)

    # break if the figure is closed
    if not plt.fignum_exists(fig.number): running = False

b1.shutdown()
b2.shutdown()

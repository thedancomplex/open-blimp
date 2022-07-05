import matplotlib.pyplot as plt
import numpy as np
import time

from pyBlimp.blimp import *
from utils.js_utils import JoyStick_helper

# setup serial device
port = "/dev/ttyUSB0"
ser = create_serial(port)

# build the blimp objects
pi1_ports = [1111, 1112, 1113]
pi2_ports = [2221, 2222, 2223]
b1 = Blimp(1, port, ser, pi1_ports)
b2 = Blimp(2, port, ser, pi2_ports)

# setup the joystick reader
js = JoyStick_helper()

# desired altitude holding
des_z1 = 1.5
des_z2 = 1.5

# heading rate and heading angle
kg = 0.05 
des_yw1 = 0.
des_yw2 = 0.

# show the active image
fig, axes = plt.subplots(1,1)

# main loop
while True:
    # handle the joystick
    ax, b1_active, b2_active = js.get_state()
     
    cmd1 = np.zeros(4)
    cmd2 = np.zeros(4)    

    if b1_active:
        cmd1[0] = -0.1*ax[1]
        cmd1[1] = -0.1*ax[0]
        cmd1[2] = 0.1*ax[3]
        cmd1[3] = 0.1*ax[2]
        b1.step(cmd1)
        time.sleep(0.001)
        
        b1.poll_image()
        if b1.pi.dist_new:
            axes.clear()
            axes.imshow(b1.I)
            axes.set_title("Blimp 1 View")
            plt.draw(); plt.pause(0.001)
        
    elif b2_active:    
        cmd2[0] = -0.1*ax[1]
        cmd2[1] = -0.1*ax[0]
        cmd2[2] = 0.1*ax[3]
        cmd2[3] = 0.1*ax[2]
        b2.step(cmd2)
            
        b2.poll_image()
        if b2.pi.dist_new:
            axes.clear()
            axes.imshow(b2.I)
            axes.set_title("Blimp 2 View")
            plt.draw(); plt.pause(0.001)
        

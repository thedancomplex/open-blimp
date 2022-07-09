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
des_z1 = 0.5
des_z2 = 0.5

# heading rate and heading angle
kg = 0.05 
des_yw1 = 0.
des_yw2 = 0.

# show the active image
fig, axes = plt.subplots(1,1)
axes.set_xticks([])
axes.set_yticks([])

# state flags
b1_first = False
b2_first = False

# setup exit on ctrl-c
running = True
def exit_handle(signum, frame):
    global running
    running = False
    
signal.signal(signal.SIGINT, exit_handle)


# main loop
while running:
    # handle the joystick
    ax, b1_active, b2_active = js.get_state()
     
    if b1_active:
        if not b1_first:
            b1_first = True
            b1.zero_z_rot()

        # update states from the pi
        b1.poll()

        # commit velocity control  
        des_vx = -ax[1]
        des_vy = -ax[0]
        des_yw1 += kg*ax[2]
        des_yw1 += 2*np.pi*(des_yw1 < -np.pi) - 2*np.pi*(des_yw1 > np.pi)

        b1.set_vel([des_vy, des_vx])
        b1.set_heading(des_yw1)

        # commit altitude control
        des_z1 += kg*ax[3]
        des_z1 = min(max(des_z1, 0.5), 2.0)
        b1.set_alt(des_z1, positive_only=True)
        
        # show image
        axes.clear()
        axes.imshow(b1.I)
        axes.set_title("Blimp 1 View")
        axes.set_xticks([])
        axes.set_yticks([])

        b1.step()

            
    elif b2_active:   
        if not b2_first:
            b2_first = True
            b2.zero_z_rot()

        # update states from the pi
        b2.poll()
        [r, p, yw] = b2.euler()
        
        # commit velocity control  
        des_vx = -ax[1]
        des_vy = -ax[0]
        des_yw2 += kg*ax[2]
        des_yw2 += 2*np.pi*(des_yw2 < -np.pi) - 2*np.pi*(des_yw2 > np.pi)

        b2.set_vel([des_vy, des_vx])
        b2.set_heading(des_yw2)

        # commit altitude control
        des_z2 += kg*ax[3]
        des_z2 = min(max(des_z2, 0.5), 2.0)
        b2.set_alt(des_z2, positive_only=True)

        # show image
        axes.clear()
        axes.imshow(b2.I)
        axes.set_title("Blimp 2 View")
        axes.set_xticks([])
        axes.set_yticks([])

        b2.step()

    else:        
        b1_first = False
        b2_first = False

    plt.draw()        
    plt.pause(0.0001)

# cleanup after the experiment is finished
b1.pi.shutdown()
b2.pi.shutdown()

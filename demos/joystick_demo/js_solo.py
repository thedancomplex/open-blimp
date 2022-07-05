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
     
    if b1_active:
        if b1.pi.bno_new:
            # update states from the pi
            b1.poll_bno()

            # commit velocity control  
            des_vx = -ax[1]
            des_vy = -ax[0]
            des_yw1 += kg*ax[2]
            des_yw1 += 2*np.pi*(des_yw1 < -np.pi) - 2*np.pi*(des_yw1 > np.pi)

            b1.set_vel([des_vy, des_vx])
            b1.set_heading(des_yw1)

        if b1.pi.dist_new:
            # update states from the pi
            b1.poll_dist()

            # commit altitude control
            des_z1 += kg*ax[3]
            des_z1 = min(max(des_z1, 0.5), 2.0)
            b1.set_alt(des_z1, positive_only=True)
        
        if b1.pi.cam_new:
            b1.poll_image()
            axes.clear()
            axes.imshow(b1.I)
            axes.set_title("Blimp 1 View")
            plt.draw()
            
    elif b2_active:    
        if b2.pi.bno_new:
            # update states from the pi
            b2.poll_bno()

            # commit velocity control  
            des_vx = -ax[1]
            des_vy = -ax[0]
            des_yw2 += kg*ax[2]
            des_yw2 += 2*np.pi*(des_yw2 < -np.pi) - 2*np.pi*(des_yw2 > np.pi)

            b2.set_vel([des_vy, des_vx])
            b2.set_heading(des_yw2)

        if b2.pi.dist_new:
            # update states from the pi
            b2.poll_dist()

            # commit altitude control
            des_z2 += kg*ax[3]
            des_z2 = min(max(des_z2, 0.5), 2.0)
            b2.set_alt(des_z2, positive_only=True)

        if b2.pi.cam_new:
            b2.poll_image()
            axes.clear()
            axes.imshow(b2.I)
            axes.set_title("Blimp 2 View")
            plt.draw()
            
    else:
        b1.set_alt(des_z1, positive_only=True)
        b2.set_alt(des_z2, positive_only=True)

    # send the command
    b1.step()
    b2.step()
    plt.pause(0.001)

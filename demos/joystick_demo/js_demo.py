import time
import numpy as np
from pyBlimp.blimp import Blimp
from utils.js_utils import JoyStick_helper

# set serial port
port = "/dev/ttyUSB0"

# build the blimp object
b = Blimp(port, logger=True)

# build the pygame joystick reader
js = JoyStick_helper()

# heading rate and heading angle
kg = 0.05 
des_yw = 0.

z_des = 0.95
control_on = False
ctrl_u = []
T = 2500
for t in range(T):
    # handle the joystick
    ax, toggle_state = js.get_state()
 
    # toggle the autopilot
    if toggle_state: 
        b.zero_z_rot()
        print("Autopilot: ON")

    else:
        print("Autopilot: OFF")

    # store joystick input for post-processing
    ctrl_u.append(np.array(ax))
    
    cmd = None
    if control_on:
        if b.pi.bno_new:
            # update states from the pi
            b.poll_bno()

            # commit velocity control  
            des_vx = -lud
            des_vy = -llr
            des_yw += kg*rlr
            des_yw += 2*np.pi*(des_yw < -np.pi) - 2*np.pi*(des_yw > np.pi)

            b.set_vel([des_vy, des_vx])
            b.set_heading(des_yw)

        if b.pi.dist_new:
            # update states from the pi
            b.poll_dist()

            # commit altitude control
            b.set_alt(z_des, positive_only=True)

    else:
        # commit manual control
        cmd = 0.09*np.array([-ax[1], ax[0], ax[3], 0.0, 0.0, ax[2]])

    # send the command
    b.step(cmd)
    time.sleep(0.05)
    
ctrl_u = np.stack(ctrl_u).T
b.save("data", ctrl_u)

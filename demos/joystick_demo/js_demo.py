import time
import numpy as np
from pyBlimp.blimp import Blimp
from utils.js_utils import JoyStick_helper

# set serial port
port = "xxx"

# build the blimp object
b = Blimp(port, logger=True)

# setup the joystick reader
js = JoyStick_helper()

# desired altitude holding
des_z = 1.

# heading rate and heading angle
kg = 0.05 
des_yw = 0.

# joystick data for data saving
ctrl_u = []

# length of test
T = 2500
for t in range(T):
    # handle the joystick
    ax, button_event, toggle_state = js.get_state()
 
    # toggle the autopilot
    if button_event and toggle_state: 
        b.zero_z_rot()
        print("Autopilot: ON")

    elif button_event and not toggle_state:
        print("Autopilot: OFF")

    # store joystick input for post-processing
    ctrl_u.append(np.array(ax))
    
    cmd = None
    if toggle_state:
        if b.pi.bno_new:
            # update states from the pi
            b.poll_bno()

            # commit velocity control  
            des_vx = -ax[1]
            des_vy = -ax[0]
            des_yw += kg*ax[2]
            des_yw += 2*np.pi*(des_yw < -np.pi) - 2*np.pi*(des_yw > np.pi)

            b.set_vel([des_vy, des_vx])
            b.set_heading(des_yw)

        if b.pi.dist_new:
            # update states from the pi
            b.poll_dist()

            # commit altitude control
            des_z += kg*ax[3]
            des_z = min(max(des_z, 0.5), 1.1)
            b.set_alt(des_z, positive_only=True)

    else:
        # commit manual control
        cmd = 0.14*np.array([-ax[1], ax[0], ax[3], ax[2]])

    # send the command
    b.step(cmd)
    time.sleep(0.05)
    
ctrl_u = np.stack(ctrl_u).T
b.save("data", ctrl_u)

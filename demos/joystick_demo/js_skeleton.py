import time
import numpy as np
from pyBlimp.blimp import Blimp
from utils.js_utils import JoyStick_helper

# set serial port 
port = ""

# build the blimp object
b = Blimp(port, logger=True)

# setup the joystick reader
js = JoyStick_helper()

# desired altitude holding
des_z = 1.

# joystick data for data saving
ctrl_u = []

# length of test
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

            # ----- add code here ----- # 
            
            des_vx = 0.
            des_vy = 0.
            des_yw = 0.
            
            # ----- end code addition ----- #
            
            # wraps the angle of the desired yaw
            des_yw += 2*np.pi*(des_yw < -np.pi) - 2*np.pi*(des_yw > np.pi)
            
            # computes the input to track the desired states
            b.set_vel([des_vy, des_vx])
            b.set_heading(des_yw)

        if b.pi.dist_new:
            # update states from the pi
            b.poll_dist()

            # ----- add code here ----- #
            
            des_z = 0.95

            # ----- end code addition ----- #
            
            # commit altitude control
            b.set_alt(des_z, positive_only=True)

    else:
        # commit manual control
        cmd = 0.14*np.array([-ax[1], ax[0], ax[3], 0.0, 0.0, ax[2]])

    # send the command
    b.step(cmd)
    time.sleep(0.05)
    
ctrl_u = np.stack(ctrl_u).T
b.save("data", ctrl_u)

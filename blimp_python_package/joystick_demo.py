import time
import numpy as np
import pygame
from blimp import Blimp

# set serial port 
port = "COM7"

# set UDP ip information
stream_ip = "192.168.3.101"

# build the blimp object
b = Blimp(port, stream_ip, logger=False)

# build the pygame joystick reader
pygame.display.init() # ignore
pygame.joystick.init()
js = pygame.joystick.Joystick(0)
dead = 0.1
scale = 0.05

z_des = 0.5
yw_des = 0.0
control_on = False
for t in range(1000):
    # get axis values
    pygame.event.pump()
    llr, lud, rlr, rud = js.get_axis(0), js.get_axis(1), js.get_axis(2), js.get_axis(3)

    # get button value
    if js.get_button(0): control_on = not control_on

    # set deadzone and scale the input
    llr = scale*llr*(not (abs(llr) < dead))
    lud = scale*lud*(not (abs(lud) < dead))
    rlr = scale*rlr*(not (abs(rlr) < dead))
    rud = scale*rud*(not (abs(rud) < dead))

    if control_on:
        # commit angle control
        b.set_ang([lud, llr])

        yw_des += rlr
        z_des += rud

        # commit heading control
        b.set_heading(yw_des)

        # commit altitude control
        b.set_alt(z_des)

    else:
        # commit manual control
        cmd = np.array([lud, llr, rud, 0.0, 0.0, rlr])
        b.step(cmd)

    # sleep a bit
    time.sleep(0.01)

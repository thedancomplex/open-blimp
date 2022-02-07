import time
import numpy as np
import pygame
from blimp import Blimp

# set serial port 
port = "/dev/ttyUSB0"

# set UDP ip information
my_ip = "192.168.3.101"
pi_ip = "192.168.3.100"

# build the blimp object
b = Blimp(port, my_ip, pi_ip, logger=False)
time.sleep(0.5)
b.zero_xy_rot()

# build the pygame joystick reader
pygame.display.init() # ignore
pygame.joystick.init()
js = pygame.joystick.Joystick(0)
button_state = False
dead = 0.1
scale1 = 0.09
scale2 = 0.8

des_vx = 0.
des_vy = 0.
des_yw = 0.

z_des = 1.0
control_on = False
while True:
    # get axis values
    pygame.event.pump()
    llr, lud, rlr, rud = js.get_axis(0), js.get_axis(1), js.get_axis(2), js.get_axis(3)

    # get button value
    if js.get_button(0) and not button_state: 
      button_state = True
      control_on = not control_on
      b.zero_z_rot()
      print("Control set to:", control_on)

    elif not js.get_button(0) and button_state:
      button_state = False

    # set deadzone and scale the input
    llr = llr*(not (abs(llr) < dead))
    lud = lud*(not (abs(lud) < dead))
    rlr = rlr*(not (abs(rlr) < dead))
    rud = rud*(not (abs(rud) < dead))

    if control_on:
        if b.pi.cam_new:
            # update image from the pi
            b.poll_image()

            # update desired state based on camera image
            # - des_pos should be a numpy array (size 3)
            des_pos = filter_image(b.I)

            # compute des_vx, des_vy, des_yw from des_x
            # - only go forward if we're aligned towards the target
            if abs(des_pos[1]) < 0.5:
                des_vx = 0.8

            # align yaw towards the target
            des_yw = des_pos[1]            

        if b.pi.bno_new:
            # update states from the pi
            b.poll_bno()

            # commit velocity control  
            des_vx += kg*(scale2*lud - des_vx)
            des_vy += kg*(scale2*llr - des_vy)
            des_yw += kg*(rlr)
            des_yw += 2*np.pi*(des_yw < -np.pi) - 2*np.pi*(des_yw > np.pi)
            b.set_vel([-des_vy, -des_vx])

            # commit heading control
            b.set_heading(des_yw)

        if b.pi.dist_new:
            # update states from the pi
            b.poll_dist()

            # commit altitude control        
            b.set_alt(z_des, positive_only=True)

        b.step()

    else:
        # commit manual control
        cmd = scale1*np.array([lud, -llr, rud, 0.0, 0.0, rlr])
        b.step(cmd)

    # sleep a bit
    time.sleep(0.05)

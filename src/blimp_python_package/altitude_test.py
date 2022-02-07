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

z_des = 0.8
control_on = False
while True:
    # get axis values
    pygame.event.pump()

    # get button value
    if js.get_button(0) and not button_state: 
      button_state = True
      control_on = not control_on
      print("Control set to:", control_on)

    elif not js.get_button(0) and button_state:
      button_state = False

    if control_on:
        if b.pi.dist_new:
            # update states from the pi
            b.poll_dist()

            # commit altitude control        
            b.set_alt(z_des, positive_only=True)
            b.step()

            # output most recent altitude readings
            print("Altitude:", str(b.x[2]))

    # sleep a bit
    time.sleep(0.01)

import time
import numpy as np
from pyBlimp.blimp import Blimp
import utils.ir_filtering

# set serial port 
port = ""

# build the blimp object
b = Blimp(port, logger=True)

des_vx = 0.
des_vy = 0.
des_yw = 0.

# desired hovering altitude
z_des = 1.0

T = 800
for t in range(T):
    if b.pi.cam_new:
        # update image from the pi
        b.poll_image()

        # extract location of target from image
        ir_pos = ir_filtering.relative_position(b.I)

        x = b.get_state()
        [_, _, yw] = b.quat_to_eul(x[6:10])
        
        if ir_pos is not None:
            # ---- add code here ---- #
            
            des_vx = 0.
            des_vy = 0.
            des_yw = 0.
        
            # ---- end code addition ---- #

    if b.pi.bno_new:
        # update states from the pi
        b.poll_bno()

        # get heading
        x = b.get_state()
        [_, _, yw] = b.quat_to_eul(x[6:10])
        
        # commit velocity control  
        b.set_vel([-des_vy, -des_vx])

        # commit heading control
        b.set_heading(des_yw)
        
    if b.pi.dist_new:
        # update states from the pi
        b.poll_dist()

        # commit altitude control        
        b.set_alt(z_des, positive_only=True)
  
    # send command and sleep a bit
    b.step()
    time.sleep(0.05)

b.save("data")

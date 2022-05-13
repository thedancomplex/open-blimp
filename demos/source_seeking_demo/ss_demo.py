import time
import numpy as np
from pyBlimp.blimp import Blimp
from utils.ir_filtering import find_ir_relative_position

# set serial port 
port = "xxx"

# build the blimp object
b = Blimp(port, logger=True)

des_vx = 0. # desired forward velocity
des_vy = 0. # desired sideway velocity
des_yw = 0. # desired heading angle

des_z = 1.0
target_des = 0.9
control_on = False

T = 800
for t in range(T):
    if b.pi.cam_new:
        # update image from the pi
        b.poll_image()

        # update desired state based on camera image
        # - des_pos should be a numpy array (size 3)
        ir_pos = find_ir_relative_position(b.I)
        
        x = b.get_state()
        [_, _, yw] = b.quat_to_eul(x[6:10])
                           
        if ir_pos is not None:
          # compute des_vx, des_vy, des_yw from des_x
          # - only go forward if we're aligned towards the target
          if abs(ir_pos[0]) < 0.5:
              des_vx = target_des - ir_pos[2]
              
          # align yaw towards the target              
          des_yw = yw - ir_pos[0]            
          print("Emitter seen")
          
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
        b.set_alt(des_z, positive_only=True)
        
    b.step()

    # sleep a bit
    time.sleep(0.05)

b.save("data")

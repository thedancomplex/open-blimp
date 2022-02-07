import time
import numpy as np
from blimp import Blimp

# set serial port 
port = "/dev/ttyACM0"

# set UDP ip information
stream_ip = "192.168.1.10"

# build the blimp object
b = Blimp(port, stream_ip, logger=True)
    
for t in range(500):
    # autonomously update state in background or
    b.poll_state()
    
    # set the blimp tracking position (goal : [px, py, pz, yaw])
    goal = np.array([0.0, 0.0, 0.0, 0.0])
    b.set_wp(goal)
    
    # set the blimp tracking velocity (vel : [vx, vy, vz, vyaw])
    vel = np.array([0.0, 0.0, 0.0, 0.0])
    b.set_vel(vel)

    # commit action (optionally with an overwrite string cmd)
    cmd = ""
    b.step(cmd)

    # sleep a bit
    time.sleep(0.05)
    
# log the blimp history
b.save("/save_dir/blimp_data")

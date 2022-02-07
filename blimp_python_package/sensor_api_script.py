import time
import numpy as np
from blimp import Blimp

# set serial port 
port = "serial0"

# set UDP ip information
stream_ip = "192.168.1.10"

# build the blimp object
b = Blimp(port, stream_ip, logger=True)
    
    
    
for t in range(500):
    # autonomously update state in background or
    b.poll_state()
    
    # grab recent image (t_stamp : float, img : np.array)
    t_stamp, img = b.get_image()

    # grab recent state  (t_stamp : float, x : np.array)
    t_stamp, x = b.get_state()

    # sleep a bit
    time.sleep(0.05)
    
# log the blimp history
b.save("/save_dir/blimp_data")

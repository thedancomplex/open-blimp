import time
import numpy as np
from blimp import Blimp

# set serial port 
port = "COM7"

# set UDP ip information
stream_ip = "192.168.1.10"

# build the blimp object
b = Blimp(port, stream_ip, logger=False)
    
for t in range(100):
    # commit action (optionally with an overwrite string cmd)
    cmd = np.array([0.0, 0.0, np.cos(0.05*t), 0.0, 0.0, 0.0])
    b.step(cmd)

    # sleep a bit
    time.sleep(0.01)
    print("step:", t)

import matplotlib.pyplot as plt
import signal

from scipy.spatial.transform import Rotation as R
from piStreaming.rcv_pi02 import MultiRcv

# setup the UDP receiver to receive images over port 8485 and BNO over port 8486
ports = (1111, 1112, 1113)
pi_ = MultiRcv(ports)

# show the image
fig, ax = plt.subplots(1,1)
    
# setup exit on ctrl-c
running = True
def exit_handle(signum, frame):
    global running
    running = False

signal.signal(signal.SIGINT, exit_handle)

while running:
    # query image
    _, img = pi_.get_image()
    ax.clear()
    ax.imshow(img)
    plt.draw(); plt.pause(0.001)

    # query imu
    # bno_stamp, bno = pi_.get_bno()
    # q = bno[0]
    # [r, p, yw] = R.from_quat(q).as_euler('xyz')
    # print(bno_stamp, "Roll:", r)

    # query distance
    # dis_stamp, dis = pi_.get_dis()
    # print(dis_stamp, "Dist:", dis, "(m)")

pi_.shutdown()

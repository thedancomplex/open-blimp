import matplotlib.pyplot as plt
import signal

from piStreaming.rcv_pi02 import MultiRcv
from pyBlimp.utils import read_config

# load desired configs
cfg = read_config("config.yaml")

# build the pi object
pi_ = MultiRcv(cfg)

# show the image
fig, ax = plt.subplots(1,1)
    
# setup exit on ctrl-c
running = True
def exit_handle(signum, frame):
    global running
    running = False

signal.signal(signal.SIGINT, exit_handle)

n = 0
while running:
    # query image
    _, I = pi_.get_image()
    ax.clear()
    ax.imshow(I)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title("FPV")
    plt.draw(); plt.pause(0.02)
    plt.imsave('saved_video/'+str(n).zfill(6)+'.jpg', I)
    n += 1

    # break if the figure is closed
    if not plt.fignum_exists(fig.number): running = False

pi_.shutdown()

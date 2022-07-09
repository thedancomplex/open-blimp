import matplotlib.pyplot as plt
import signal

from piStreaming.rcv_pi02 import MultiRcv


# setup the UDP receiver to receive images over port 8485 and BNO over port 8486
im_sz = (240, 360, 3)
ports = (1111, 1112, 1113)
pi_ = MultiRcv(ports, im_sz)

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
    _, I = pi_.get_image()
    ax.clear()
    ax.imshow(I)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title("FPV")
    plt.draw(); plt.pause(0.0001)

    # break if the figure is closed
    if not plt.fignum_exists(fig.number): running = False

pi_.shutdown()

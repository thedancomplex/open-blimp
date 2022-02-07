#!/usr/bin/env/python3
# default python libraries
import time
import numpy as np
from threaded_stream_rcv import ThreadedPiStream

# testing results:
# - img hz: ~5hz
# - BNO hz: ~30hz
# - VL5 hz: ~6.2hz

# example for using the asynchronous receiver
if __name__ == "__main__":
  # setup the UDP receiver to receive images over port 8485 and BNO over port 8486
  pi_ = ThreadedPiStream(8485, 8486, 8487, debug=False)

  # constantly query for newly received data
  t0 = 3*[time.time()]
  hzh = 3*[np.zeros(128)]

  k = 0
  while True:
    # query image
    if pi_.cam_new:
        img_stamp, image = pi_.get_image()
        hzh[0] = np.roll(hzh[0], 1)
        hzh[0][0] = time.time()-t0[0]
        t0[0] = time.time()

    # query imu
    if pi_.bno_new:
        bno_stamp, bno = pi_.get_bno()
        hzh[1] = np.roll(hzh[1], 1)
        hzh[1][0] = time.time()-t0[1]
        t0[1] = time.time()

    # query distance
    if pi_.dist_new:
        dist_stamp, dist = pi_.get_dist()
        hzh[2] = np.roll(hzh[2], 1)
        hzh[2][0] = time.time()-t0[2]
        t0[2] = time.time()

    if k % 10000 == 0:
        print("Img, BNO, VL53 hz:", 1./np.mean(hzh[0]), 1./np.mean(hzh[1]), 1./np.mean(hzh[2]))
        pass

    k += 1

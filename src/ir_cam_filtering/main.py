import time
import numpy as np
import cv2 as cv
from filter import *
from pi_zero_w_streaming.threaded_stream_rcv import ThreadedPiStream

pi_ = ThreadedPiStream(8485, 8486, debug=False)
time.sleep(1)
_, img = pi_.get_image()

bin_img = filter_image(img)
c, r = estimate_circle(bin_img)
c_img = draw_image(img, bin_img, c, r)

while True:
    # show image
    cv.imshow("image", c_img)

    # grab/process next image
    _, img = pi_.get_image()

    bin_img = filter_image(img)
    c, r = estimate_circle(bin_img)
    c_img = draw_image(img, bin_img, c, r)
    print("Radius:", r)

    cv.waitKey(20)

print("Finished!")

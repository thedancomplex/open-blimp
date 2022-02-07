import numpy as np
import cv2 as cv
from filter import *

vid = cv.VideoCapture('output.avi')

success, img = vid.read()
bin_img = filter_image(img)
c, r = estimate_circle(bin_img)
c_img = draw_image(img, bin_img, c, r)

out = cv.VideoWriter('output_filtered.avi',cv.VideoWriter_fourcc('M','J','P','G'), 25, (c_img.shape[1], c_img.shape[0]))

while success:
    # show/write images
    cv.imshow("image", c_img)
    out.write(c_img)

    # grab/process next image
    success, img = vid.read()
    bin_img = filter_image(img)
    c, r = estimate_circle(bin_img)
    c_img = draw_image(img, bin_img, c, r)

    cv.waitKey(20)

print("Finished!")

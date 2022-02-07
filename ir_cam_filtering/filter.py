import numpy as np
import cv2 as cv

def filter_image(img_):
    # get grayscale image
    img = cv.cvtColor(img_, cv.COLOR_BGR2GRAY)

    # get the initial binary image
    mask = img > 160
    bin_img = np.zeros((128,128))
    bin_img[mask] = 255

    # perform blurring
    bin_img = cv.GaussianBlur(bin_img, ksize=(11,11), sigmaX=9)
    mask = bin_img > 0
    bin_img = np.zeros((128,128))
    bin_img[mask] = 255

    # perform open/close operations
    N = 3
    #bin_img = cv.open(bin_img, None, iterations=N)
    bin_img = cv.erode(bin_img, None, iterations=N)

    return bin_img.astype(np.uint8)

def scale_img(img, sc=6):
    w, h = int(img.shape[1]*sc), int(img.shape[0]*sc)
    dim = (w, h)
    return cv.resize(img, dim)

def estimate_circle(img):
    contours = cv.findContours(img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)[0]
    if len(contours) == 0: return None, None

    c, r = cv.minEnclosingCircle(contours[0])
    return c, r

def draw_image(im1, bin_im, c, r):
    bin_cim = cv.cvtColor(bin_im, cv.COLOR_GRAY2BGR)
    if c is not None: bin_cim = cv.circle(bin_cim, (int(c[0]), int(c[1])), int(r), (0,255,0),1)

    c_img = np.concatenate((bin_cim, im1), axis=1)
    return scale_img(c_img)

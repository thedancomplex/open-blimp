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
    bin_img = cv.erode(bin_img, None, iterations=3)
    return bin_img.astype(np.uint8)

def scale_img(img, sc=6):
    w, h = int(img.shape[1]*sc), int(img.shape[0]*sc)
    dim = (w, h)
    return cv.resize(img, dim)

def estimate_circle(img):
    contours = cv.findContours(img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)[1]
    if len(contours) == 0 or np.sum(contours) == 0: return None, None

    c, r = cv.minEnclosingCircle(contours[0])
    
    return c, r

def draw_circle_on_image(im):
    bin_im = filter_image(im)
    c, r = estimate_circle(bin_im)
    if c is not None:    
      cim = cv.circle(im, (int(c[0]), int(c[1])), int(r), (0,255,0),1)
    else:
      cim = im
      
    return cim

def find_ir_relative_position(img):
    w, h, _ = img.shape
    bin_img = filter_image(img)
    c, r = estimate_circle(bin_img)
    z = 0
    NoneType = type(None)
    if not type(c) == NoneType:
        x, y = c
        x -= w/2.
        y -= h/2.
        
        # convert pixel coordinates to angles
        pixel2ang = 0.0189*0.766
        x *= pixel2ang
        y *= pixel2ang
        
        # exponential distance function fit from data (transformed to cm)
        z = 21.5*r**(-1.21)*30.48
        return ([-x, y, z/100])

    
    return None


   

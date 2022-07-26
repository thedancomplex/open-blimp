import cv2 as cv
import numpy as np
        
def combine(I1, I2, final_cv=True):
    cv_I1 = cv.cvtColor(np.uint8(255*I1), cv.COLOR_RGB2BGR)
    cv_I2 = cv.cvtColor(np.uint8(255*I2), cv.COLOR_RGB2BGR)
    frame = cv.hconcat([cv_I1, cv_I2])
    
    if final_cv: return frame
    else: return cv.cvtColor(frame, cv.COLOR_BGR2RGB)/255.

def cv2np(I):
    return cv.cvtColor(I, cv.COLOR_BGR2RGB)/255.

def np2cv(I):
    return cv.cvtColor(np.uint8(255*I), cv.COLOR_RGB2BGR)
    
def resize(I, scale, retain_edges=True):
    h, w, c = I.shape
    if retain_edges:
        I_ = cv.resize(I, (int(w*scale), int(h*scale)), interpolation=cv.INTER_NEAREST)
    else:
        I_ = cv.resize(I, (int(w*scale), int(h*scale)))
    
    return I_
    
def sharpen(I):
    kernel = np.array([[0,-1,0], [-1,5,-1], [0,-1,0]])
    return cv.filter2D(I, -1, kernel)

import cv2 as cv
import numpy as np

from pupil_apriltags import Detector

class AprilTagHelper:
    def __init__(self, K_path, sz=0.8875):
        self.K = np.loadtxt(K_path).tolist()
        self.at = Detector(families='tag36h11',
                           nthreads=8,
                           quad_decimate=1,
                           quad_sigma=0.0,
                           refine_edges=1,
                           decode_sharpening=0.,
                           debug=0)

        # paper tags are 0.202
        # poster tags are 0.8875
        self.sz = sz
        
    def detect(self, img, label=False):
        # prep image for detection
        cv_img = cv.cvtColor(np.uint8(255*img), cv.COLOR_RGB2BGR)
        gray = cv.cvtColor(cv_img, cv.COLOR_BGR2GRAY)

        # detect tags
        tags = self.at.detect(gray, estimate_tag_pose=True,
                              camera_params=self.K, tag_size=self.sz)

        # parse tag info
        if label:
            for tag in tags:
                cv_img = cv.circle(cv_img, np.int32(tag.center), 1, (0,255,0), 3)
                for i in range(4):
                    corner = tag.corners[i,:]
                    cv_img = cv.circle(cv_img, np.int32(corner), 1, (0,0,255), 3)

            img = cv.cvtColor(cv_img, cv.COLOR_BGR2RGB)/255.
            return tags, img

        return tags

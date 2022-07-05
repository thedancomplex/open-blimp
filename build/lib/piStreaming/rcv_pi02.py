#!/usr/bin/env/python3
# default python libraries
import io
import socket
import struct
import sys
import threading
import time

# check camera related libraries
try:
    import cv2
    import numpy as np

except:
    pass

class MultiRcv:
    def __init__(self, cam_port, bno_port, dist_port, debug=False):
        self.img = None
        self.gyro = [0., 0., 0.]
        self.quat = [0., 0., 0., 1.]
        self.acc = [0., 0., 0.]
        self.dist = 0.
        
        self.cam_stamp = None
        self.bno_stamp = None
        self.dist_port = None

        # warning flags
        self.debug = debug
        self.cam_hz = []
        self.bno_hz = []
        self.dist_hz = []
        self.cam_new = False
        self.bno_new = False
        self.dist_new = False
        self.cam_at_least_one = False
        self.bno_at_least_one = False
        self.dist_at_least_one = False
        
        # thread-check
        self.main_thread = threading.currentThread()

        # start camera listener
        cam_libs = 'cv2' in sys.modules and 'numpy' in sys.modules
        if cam_port is not None and cam_libs:
            self.cam_sock = socket.socket()
            self.cam_sock.bind(('', cam_port))
            self.cam_thread = threading.Thread(target=self.handle_cam_read, args=())
            self.cam_thread.start()

        else:
            print("Camera setup skipped")

        # start bno listener
        if bno_port is not None:
            self.bno_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.bno_sock.bind(('', bno_port))
            self.bno_thread = threading.Thread(target=self.handle_bno_read, args=())
            self.bno_thread.start()

        else:
            print("BNO055 setup skipped")

        # start distance sensor listener
        if dist_port is not None:
            self.dist_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.dist_sock.bind(('', dist_port))
            self.dist_thread = threading.Thread(target=self.handle_dist_read, args=())
            self.dist_thread.start()

        else:
            print("VL53L0X setup skipped")

    def get_image(self):
        # check if camera thread is active
        assert hasattr(self, 'cam_thread'), "Camera thread not initialized"

        self.cam_new = False
        return self.cam_stamp, self.img

    def get_bno(self):
        # check if bno thread is active
        assert hasattr(self, 'bno_thread'), "BNO thread not initialized"

        self.bno_new = False
        return self.bno_stamp, (self.quat, self.gyro, self.acc)

    def get_dist(self):
        #check if distance thread is active
        assert hasattr(self, 'dist_thread'), "Distance thread not initialized"

        self.dist_new = False
        return self.dist_stamp, self.distance/100.

    def handle_cam_read(self):
        self.cam_sock.listen(0)
        self.cam_connection = self.cam_sock.accept()[0].makefile('rb')

        t0 = time.time()
        sz = 32
        thz = np.zeros(sz)
        idz = 0
        while self.main_thread.is_alive():
            image_len = struct.unpack('<L', self.cam_connection.read(struct.calcsize('<L')))[0]

            image_stream = io.BytesIO()
            image_stream.write(self.cam_connection.read(image_len))
            image_stream.seek(0)

            # parse image
            nparr = np.frombuffer(image_stream.read(), np.uint8)
            imgBGR = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            self.img = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2RGB)/255.

            # set timestamp
            self.cam_stamp = time.time()
            self.cam_new = True
            self.cam_warn = False
            self.cam_at_least_one = True

            if self.debug:
                thz[idz] = time.time()-t0
                t0 = time.time()
                idz += 1
                idz = idz % sz
                self.cam_hz.append(1./np.mean(thz))

    def handle_bno_read(self):
        t0 = time.time()
        sz = 32
        thz = np.zeros(sz)
        idz = 0
        while self.main_thread.is_alive():
            data, addr = self.bno_sock.recvfrom(100)
            if data:
                bno = struct.unpack("<10d", data)
                self.quat = np.array(bno[0:4])
                self.gyro = np.array(bno[4:7])*np.pi/180
                self.acc  = np.array(bno[7:])
                self.bno_stamp = time.time()
                self.bno_new = True
                self.bno_warn = False
                self.bno_at_least_one = True

            if self.debug:
                thz[idz] = time.time()-t0
                t0 = time.time()
                idz += 1
                idz = idz % sz
                self.bno_hz.append(1./np.mean(thz))

    def handle_dist_read(self):
        t0 = time.time()
        sz = 32
        thz = np.zeros(sz)
        idz = 0
        while self.main_thread.is_alive():
            data, addr = self.dist_sock.recvfrom(100)

            if data:
                distance = struct.unpack("<1d", data)
                self.distance = distance[0]
                self.dist_stamp = time.time()
                self.dist_new = True
                self.dist_warn = False
                self.dist_at_least_one = True

            if self.debug:
                thz[idz] = time.time()-t0
                t0 = time.time()
                idz += 1
                idz = idz % sz
                self.dist_hz.append(1./np.mean(thz))


# example for using the asynchronous receiver
if __name__ == "__main__":
    import matplotlib.pyplot as plt

    # setup the UDP receiver to receive images over port 8485 and BNO over port 8486
    pi_ = MultiRcv(8485, 8486, 8487, debug=False)

    # constantly query for newly received data
    t0 = 3*[time.time()]
    hzh = 3*[np.zeros(32)]

    fig, ax = plt.subplots(1,1)
    while True:
        # query image
        if pi_.cam_new and not pi_.debug:
            _, img = pi_.get_image()
            ax.clear()
            ax.imshow(img)
            plt.draw(); plt.pause(0.001)

        # query imu
        if pi_.bno_new and not pi_.debug:
            _, bno = pi_.get_bno()
            #print("Quat:", bno[0])

        # query distance
        if pi_.dist_new and not pi_.debug:
            _, dist = pi_.get_dist()
            print("Dist:", dist, "(m)")


        # output/debug statements
        if pi_.debug:
            print(" ---- DEBUG ---- ")
            if pi_.cam_at_least_one: 
                print("Cam Hz:", pi_.cam_hz[-1])

            if pi_.bno_at_least_one:
                print("BNO Hz:", pi_.bno_hz[-1])

            if pi_.dist_at_least_one:
                print("VL53 Hz:", pi_.dist_hz[-1])
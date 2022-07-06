#!/usr/bin/env/python3
# default python libraries
import cv2
import io
import multiprocessing as mp
import numpy as np
import socket
import struct
import threading
import time

from multiprocessing import shared_memory as sm

class MultiRcv:
    def __init__(self, ports):
        self.ports = ports

        # size of incoming image
        self.im_sz = (240, 360, 3)

        # create shared memory for mp
        self.sh_img = sm.SharedMemory(create=True, size=np.prod(self.im_sz))
        self.sh_bno = sm.SharedMemory(create=True, size=80)
        self.sh_dis = sm.SharedMemory(create=True, size=8)
        self.sh_img_stamp = sm.SharedMemory(create=True, size=8)
        self.sh_bno_stamp = sm.SharedMemory(create=True, size=8)
        self.sh_dis_stamp = sm.SharedMemory(create=True, size=8)

        # interfaces for accessing shared memory
        self.img = np.ndarray(self.im_sz, dtype=np.uint8, buffer=self.sh_img.buf)
        self.bno = np.ndarray(10, dtype=np.double, buffer=self.sh_bno.buf)
        self.dis = np.ndarray(1, dtype=np.double, buffer=self.sh_dis.buf)
        self.img_stamp = np.ndarray(1, np.double, buffer=self.sh_img_stamp.buf)
        self.bno_stamp = np.ndarray(1, np.double, buffer=self.sh_bno_stamp.buf)
        self.dis_stamp = np.ndarray(1, np.double, buffer=self.sh_dis_stamp.buf)

        # create shared memory for shutting sensor process down
        self.sh_flag = sm.SharedMemory(create=True, size=1)
        self.flag = np.ndarray(1, dtype=np.bool_, buffer=self.sh_flag.buf)
        self.flag[0] = True
        
        # make proper quaternion
        self.bno[3] = 1.

        # setup process to handle sensor stream
        self.sm_names = (self.sh_img.name, self.sh_img_stamp.name, \
                         self.sh_bno.name, self.sh_bno_stamp.name, \
                         self.sh_dis.name, self.sh_dis_stamp.name, \
                         self.sh_flag.name)

        self.psensor = mp.Process(target=self.handle_sensor_reads, args=())

        # start the process
        self.psensor.start()

    def shutdown(self):
        self.flag[0] = False
        self.psensor.join()

        self.sh_img.close()
        self.sh_bno.close()
        self.sh_dis.close()
        self.sh_img_stamp.close()
        self.sh_bno_stamp.close()
        self.sh_dis_stamp.close()
        self.sh_flag.close()
        
        self.sh_img.unlink()
        self.sh_bno.unlink()
        self.sh_dis.unlink()
        self.sh_img_stamp.unlink()
        self.sh_bno_stamp.unlink()
        self.sh_dis_stamp.unlink()
        self.sh_flag.unlink()
        
    def get_image(self):
        return self.img_stamp, self.img/255.

    def get_bno(self):
        q = self.bno[0:4]
        g = self.bno[4:7]
        a = self.bno[7:]
        return self.bno_stamp, (q, g, a)

    def get_dis(self):
        return self.dis_stamp, self.dis[0]/100.

    def handle_sensor_reads(self):
        # instantiate threads to handle sensor stream
        rcv = ThreadedRcv(self.ports, self.im_sz, self.sm_names)

class ThreadedRcv:
    def __init__(self, ports, im_sz, sm_names):        
        self.ports = ports
        self.im_sz = im_sz
        self.sm_names = sm_names
        
        # setup the shared memory
        self.sh_img = sm.SharedMemory(sm_names[0])
        self.sh_bno = sm.SharedMemory(sm_names[2])
        self.sh_dis = sm.SharedMemory(sm_names[4])
        self.sh_img_stamp = sm.SharedMemory(sm_names[1])
        self.sh_bno_stamp = sm.SharedMemory(sm_names[3])
        self.sh_dis_stamp = sm.SharedMemory(sm_names[5])

        # interfaces for accessing shared memory
        self.img = np.ndarray(im_sz, dtype=np.uint8, buffer=self.sh_img.buf)
        self.bno = np.ndarray(10, dtype=np.double, buffer=self.sh_bno.buf)
        self.dis = np.ndarray(1, dtype=np.double, buffer=self.sh_dis.buf)
        self.img_stamp = np.ndarray(1, np.double, buffer=self.sh_img_stamp.buf)
        self.bno_stamp = np.ndarray(1, np.double, buffer=self.sh_bno_stamp.buf)
        self.dis_stamp = np.ndarray(1, np.double, buffer=self.sh_dis_stamp.buf)

        # setup threads
        self.img_thread = threading.Thread(target=self.handle_img_read, args=())        
        self.bno_thread = threading.Thread(target=self.handle_bno_read, args=())
        self.dis_thread = threading.Thread(target=self.handle_dis_read, args=())

        self.img_thread.start()
        self.bno_thread.start()
        self.dis_thread.start()
                        
    def handle_img_read(self):
        # setup socket
        sock = socket.socket()
        sock.bind(('0.0.0.0', self.ports[0]))
        sock.listen(0)
        img_connection = sock.accept()[0].makefile('rb')

        # setup running flag 
        sh_flag = sm.SharedMemory(self.sm_names[-1])
        running = np.ndarray(1, dtype=np.bool_, buffer=sh_flag.buf)

        while running:
            # read image size
            image_len = struct.unpack('<L', img_connection.read(struct.calcsize('<L')))[0]

            # read image data
            image_stream = io.BytesIO()
            image_stream.write(img_connection.read(image_len))
            image_stream.seek(0)

            # parse image
            nparr = np.frombuffer(image_stream.read(), np.uint8)
            imgBGR = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            self.img[:,:,:] = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2RGB)

            # read current time
            tframe = np.frombuffer(img_connection.read(8), np.double)
            self.img_stamp[0] = tframe[0]

        sh_flag.close()
        self.sh_img.close()
        self.sh_img_stamp.close()
        
    def handle_bno_read(self):
        # setup socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', self.ports[1]))

        # setup running flag 
        sh_flag = sm.SharedMemory(self.sm_names[-1])
        running = np.ndarray(1, dtype=np.bool_, buffer=sh_flag.buf)
        
        while running:
            data, _ = sock.recvfrom(88)
            if data:
                new_bno = np.array(struct.unpack("<11d", data))
                self.bno[:] = new_bno[:-1]

                # convert gyro to rads/sec
                self.bno[4:7] = self.bno[4:7]*np.pi/180.
                self.bno_stamp[0] = new_bno[-1]

        sh_flag.close()
        self.sh_bno.close()
        self.sh_bno_stamp.close()
        
    def handle_dis_read(self):
        # setup socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', self.ports[2]))

        # setup running flag 
        sh_flag = sm.SharedMemory(self.sm_names[-1])
        running = np.ndarray(1, dtype=np.bool_, buffer=sh_flag.buf)

        while running:
            data, _ = sock.recvfrom(16)

            if data:
                new_dis = struct.unpack("<2d", data)
                self.dis[0] = new_dis[0]
                self.dis_stamp[0] = new_dis[1]

        sh_flag.close()
        self.sh_dis.close()
        self.sh_dis_stamp.close()

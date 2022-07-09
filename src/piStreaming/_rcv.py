import cv2
import io
import multiprocessing as mp
import numpy as np
import socket
import signal
import struct
import threading
import time

from multiprocessing import shared_memory as sm
from numpy import pi

def handle_sensors(ports, im_sz, names, locks):
    """ private sensor handler spawned by a parent MultiRcv object
        - this handle should NOT be used by an application
        - only spawned and controlled by a parent MultiRcv object    
    """
    # instantiate threads to handle sensor stream
    rcv = _Rcv(ports, im_sz, names, locks)
    rcv.shutdown()
    
class _Rcv:
    """ private sensor handler that runs on a spawned process to read data
        - this handler should NOT be used by an application
        - only spawned and controlled by a parent MultiRcv object    
    """
    
    def __init__(self, ports, im_sz, sm_names, locks):        
        self.ports = ports
        self.im_sz = im_sz
        self.sm_names = sm_names
        
        # split the locks
        self.lock_img = locks[0]
        self.lock_bno = locks[1]
        self.lock_dis = locks[2]
        
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

        # setup control-c handler
        signal.signal(signal.SIGINT, self.handler)

        self.img_thread.start()
        self.bno_thread.start()
        self.dis_thread.start()

    def shutdown(self):
        # - dummy function to wait for cleanup
        self.img_thread.join()
        self.bno_thread.join()
        self.dis_thread.join()

    def handler(self, signum, frame):
        # - ctrl-c handler to start process cleanup
        sh_flag = sm.SharedMemory(self.sm_names[-1])
        running = np.ndarray(1, dtype=np.bool_, buffer=sh_flag.buf)
        running[0] = False
        sh_flag.close()

    def handle_img_read(self):
        # - parses all image data coming from the pi and stores it
        # - into shared memory. Uses protected writes to make sure
        # - race conditions are avoided
    
        # setup running flag 
        sh_flag = sm.SharedMemory(self.sm_names[-1])
        running = np.ndarray(1, dtype=np.bool_, buffer=sh_flag.buf)

        # setup socket
        sock = socket.socket()
        sock.setblocking(0)
        sock.settimeout(0.5)
        sock.bind(('0.0.0.0', self.ports[0]))
        sock.listen(0)
        
        connected = False
        while running[0] and not connected:
            try:
                img_connection = sock.accept()[0].makefile('rb')
                connected = True
            
            except: pass

        while running[0]:
            # read image size
            try: data = img_connection.read(struct.calcsize('<L'))
            except: data = None
                
            # parse the data if available
            if data is not None:
                # ignore if wrong size
                if len(data) != struct.calcsize('<L'): continue
                image_len = struct.unpack('<L', data)[0]

                # read image data
                image_stream = io.BytesIO()
                try: img_data = img_connection.read(image_len)
                except: continue
                
                # ignore if wrong size
                if len(img_data) != image_len: continue
                
                image_stream.write(img_data)
                image_stream.seek(0)

                # parse image
                nparr = np.frombuffer(image_stream.read(), np.uint8)
                imgBGR = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                img = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2RGB)

                # read current time
                tframe = np.frombuffer(img_connection.read(8), np.double)
                img_stamp = tframe[0]

                # protected update (wait for lock to be free)
                self.lock_img.acquire()
                self.img[:,:,:] = img
                self.img_stamp[0] = img_stamp       
                self.lock_img.release()

        sh_flag.close()
        self.sh_img.close()
        self.sh_img_stamp.close()

    def handle_bno_read(self):
        # - parses all bno data coming from the pi and stores it
        # - into shared memory. Uses protected writes to make sure
        # - race conditions are avoided

        # setup socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(0)
        sock.settimeout(0.5)        
        sock.bind(('0.0.0.0', self.ports[1]))

        # setup running flag 
        sh_flag = sm.SharedMemory(self.sm_names[-1])
        running = np.ndarray(1, dtype=np.bool_, buffer=sh_flag.buf)
        
        while running[0]:
            try: data, _ = sock.recvfrom(88)
            except: data = None

            # parse if available                
            if data is not None:
                # ignore if wrong size
                if len(data) != 88: continue
                
                # parse into bno data            
                new_bno = np.array(struct.unpack("<11d", data))

                # convert gyro to rads/sec
                new_bno[4:7] = new_bno[4:7]*pi/180.

                # protected update (skip if lock isn't available)
                if self.lock_bno.acquire(False):                    
                    self.bno[:] = new_bno[:-1]
                    self.bno_stamp[0] = new_bno[-1]
                    self.lock_bno.release()
                
        sh_flag.close()
        self.sh_bno.close()
        self.sh_bno_stamp.close()
        
    def handle_dis_read(self):
        # - parses all distance data coming from the pi and stores it
        # - into shared memory. Uses protected writes to make sure
        # - race conditions are avoided

        # setup socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(0)
        sock.settimeout(0.5)        
        sock.bind(('0.0.0.0', self.ports[2]))

        # setup running flag 
        sh_flag = sm.SharedMemory(self.sm_names[-1])
        running = np.ndarray(1, dtype=np.bool_, buffer=sh_flag.buf)

        while running[0]:
            try: data, _ = sock.recvfrom(16)
            except: data = None

            # parse if available
            if data is not None:
                # ignore if wrong size
                if len(data) != 16: continue

                # parse into distance data
                new_dis = struct.unpack("<2d", data)

                # protected update (skip if lock isn't available)
                if self.lock_dis.acquire(False):
                    self.dis[0] = new_dis[0]
                    self.dis_stamp[0] = new_dis[1]
                    self.lock_dis.release()

        sh_flag.close()
        self.sh_dis.close()
        self.sh_dis_stamp.close()


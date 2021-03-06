#!/usr/bin/env/python3
# default python libraries
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

class ThreadedPiStream:
  def __init__(self, cam_port, bno_port, dist_port, debug=False):
    self.img = None
    self.gyro = None
    self.rot = None
    self.cam_stamp = None
    self.bno_stamp = None
    self.dist_port = None

    # warning flags
    self.debug = debug
    self.cam_hz = []
    self.bno_hz = []
    self.dist_hz = []

    # thread-check
    self.main_thread = threading.currentThread()

    # start camera listener
    cam_libs = 'cv2' in sys.modules and 'numpy' in sys.modules
    if cam_port is not None and cam_libs:
      self.cam_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      self.cam_sock.bind(('', cam_port))
      self.cam_thread = threading.Thread(target=self.handle_cam_read, args=())
      self.cam_thread.start()
      self.cam_new = False

    else:
      print("Camera setup skipped")

    # start bno listener
    if bno_port is not None:
      self.bno_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      self.bno_sock.bind(('', bno_port))
      self.bno_thread = threading.Thread(target=self.handle_bno_read, args=())
      self.bno_thread.start()
      self.bno_new = False

    else:
      print("BNO055 setup skipped")

    # start distance sensor listener
    if dist_port is not None:
      self.dist_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      self.dist_sock.bind(('', dist_port))
      self.dist_thread = threading.Thread(target=self.handle_dist_read, args=())
      self.dist_thread.start()
      self.dist_new = False

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
    return self.bno_stamp, (self.quat, self.gyro)

  def get_dist(self):
    #check if distance thread is active
    assert hasattr(self, 'dist_thread'), "Distance thread not initialized"
    
    self.dist_new = False
    return self.dist_stamp, self.distance

  def handle_cam_read(self):
    t0 = time.time()
    sz = 32
    thz = np.zeros(sz)
    idz = 0
    while self.main_thread.isAlive():
      data, addr = self.cam_sock.recvfrom(2000000)

      if data:
        # parse image
        nparr = np.frombuffer(data, np.uint8)
        imgBGR = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        self.img = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2RGB)

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
    while self.main_thread.isAlive():
      data, addr = self.bno_sock.recvfrom(100)
      if data:
        bno = struct.unpack("<7d", data)
        self.quat = np.array(bno[0:4])
        self.gyro = np.array(bno[4:])*np.pi/180
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
    while self.main_thread.isAlive():
      data, addr = self.dist_sock.recvfrom(100)

      if data:
        distance = struct.unpack("<1d", data)
        self.distance = distance[0]/1000.
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
  # setup the UDP receiver to receive images over port 8485 and BNO over port 8486
  pi_ = ThreadedPiStream(8485, 8486, 8487, debug=False)

  # constantly query for newly received data
  t0 = 3*[time.time()]
  hzh = 3*[np.zeros(32)]
  while True:
    # query image
    if pi_.cam_new:
        print("image!")
        img_stamp, image = pi_.get_image()
        img = image/255.
        cv2.imshow("Output", img)
        cv2.waitKey(100)
        
    # query imu
    if pi_.bno_new:
        bno_stamp, bno = pi_.get_bno()
        print("Quat:", bno[0])

    # query distance
    if pi_.dist_new:
        dist_stamp, dist = pi_.get_dist()
        print("Distance:", dist/1000, "m")
    
    
    

#!/usr/bin/env/python3
# default python libraries
import socket
import struct
import sys
import threading
import time
import matplotlib.pyplot as plt

# check camera related libraries
try:
  import cv2
  import numpy as np

except: 
  pass

class ThreadedPiStream:
  def __init__(self, cam_port, bno_port, debug=False):
    self.img = None
    self.gyro = None
    self.rot = None
    self.acc = None
    self.cam_stamp = None
    self.bno_stamp = None

    # warning flags
    self.debug = debug
    self.cam_hz = []
    self.bno_hz = []

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
      print("BNO setup skipped")

  def get_image(self):
    # check if camera thread is active
    assert hasattr(self, 'cam_thread'), "Camera thread not initialized"

    self.cam_new = False
    return self.cam_stamp, self.img

  def get_bno(self):
    # check if bno thread is active
    assert hasattr(self, 'bno_thread'), "BNO thread not initialized"

    self.bno_new = False
    return self.bno_stamp, (self.rot, self.gyro)

  def get_pair(self):
    # check threads
    assert hasattr(self, 'cam_thread'), "Camera thread not initialized"
    assert hasattr(self, 'bno_thread'), "BNO thread not initialized"

    self.cam_new = False
    self.bno_new = False
    return self.cam_stamp, self.bno_stamp, self.img, (self.rot, self.gyro, self.acc)

  def handle_cam_read(self):
    t0 = time.time()
    sz = 32
    thz = np.zeros(sz)
    idz = 0
    while True:
      data, addr = self.cam_sock.recvfrom(2000000)

      if data:
        cam_data, time_data = data[:-8], data[-8:]

        print(len(cam_data)*4)
        # parse image
        nparr = np.frombuffer(cam_data, np.uint8)
        imgBGR = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        self.img = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2RGB)

        # parse timestamp
        self.cam_stamp = struct.unpack("<d", time_data)[0]
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
    while True:
      data, addr = self.bno_sock.recvfrom(16384)

      if data:
        bno = struct.unpack("<10d", data)
        self.rot = np.array(bno[0:3])*np.pi/180
        self.gyro = np.array(bno[3:6])*np.pi/180
        self.acc = np.array(bno[6:9])
        self.bno_stamp = bno[9]
        self.bno_new = True
        self.bno_warn = False
        self.bno_at_least_one = True

        if self.debug:
          thz[idz] = time.time()-t0
          t0 = time.time()
          idz += 1
          idz = idz % sz
          self.bno_hz.append(1./np.mean(thz))

# example for using the asynchronous receiver
if __name__ == "__main__":
  # setup the UDP receiver to receive images over port 8485 and BNO over port 8486
  pi_ = ThreadedPiStream(8485, 8486, debug=True)

  fig1, ax1 = plt.subplots(1, 2)
  fig2, ax2 = plt.subplots(1, 2)
  bno_d = []
  # constantly query for newly received images
  while True:
    # query just the most recent image
    #img_stamp, image = pi_.get_image()

    # query just the next bno data
    #bno_stamp, bno = pi_.get_bno()

    # query both
    img_stamp, bno_stamp, image, bno = pi_.get_pair()
    #time.sleep(0.01)
    ax1[0].clear()
    ax1[0].plot(pi_.cam_hz, 'r-')
    ax1[0].set_ylim([0,20])

    ax1[1].clear()
    ax1[1].plot(pi_.bno_hz, 'b-')
    ax1[1].set_ylim([0,40])

    if image is not None:
      ax2[0].clear()
      ax2[0].imshow(image)

    if bno[0] is not None:
      bno_d.append(bno[0][2])
      ax2[1].clear()
      ax2[1].plot(bno_d, 'g-')

    plt.draw(); plt.pause(0.1)

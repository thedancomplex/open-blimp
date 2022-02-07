#!/usr/bin/env/python3
# default python libraries
import argparse
import socket
import struct
import threading
import time

# camera specific libraries
try:
    from picamera import PiCamera
    from picamera.array import PiRGBArray
    import numpy as np
    from io import BytesIO
    from PIL import Image

except Exception as e: print("Camera import error:", e)

# BNO specific libraries
try:
   import serial
   import adafruit_bno055

except Exception as e: print("BNO import error:", e)


class ThreadedStream:
  def __init__(self, udp_ip, only_cam=False, only_imu=False, udp_ports=(8485, 8486), uart_port="/dev/ttyS0", res=(368, 368), quality=20, fps=10, hz=30):

    # setup UDP socket
    print("Streaming to", udp_ip)
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.cam_port = udp_ports[0]
    self.bno_port = udp_ports[1]
    self.udp_ip = udp_ip
    self.quality = quality
    self.t0 = time.time()
    self.hz = hz

    try:
      # setup camera
      self.cam = PiCamera()
      self.cam.resolution = res
      self.cam.framerate = fps
      self.rawCap = PiRGBArray(self.cam, size=self.cam.resolution)

      # setup camera thread
      if not only_imu:
        self.cam_thread = threading.Thread(target=self.handle_cam_write, args=())
        self.cam_thread.start()
        print("Camera successfully registered")

    except Exception as e:
      print("Camera setup skipped:", e)

    try:
      # setup UART
      uart = serial.Serial(uart_port, 115200, timeout=10)
      self.sensor = adafruit_bno055.BNO055_UART(uart)
      self.sensor.mode = 11

      # setup bno thread
      if not only_cam:
        self.bno_thread = threading.Thread(target=self.handle_bno_write, args=())
        self.bno_thread.start()
        print("BNO successfully registered")

    except Exception as e:
      print("BNO setup skipped:", e)

  def handle_cam_write(self):
    for frame in self.cam.capture_continuous(self.rawCap, format="bgr", use_video_port=True):
      buf = BytesIO()
      img = frame.array
      self.rawCap.truncate(0); t1 = time.time()
      img = Image.fromarray(np.uint8(img)).convert('RGB')

      # store compressed image
      img.save(buf, "JPEG", quality=self.quality)

      # store timestamp
      t0_bytes = struct.pack("<d", time.time()-self.t0)
      buf.write(t0_bytes); #print(buf.getbuffer().nbytes)
      buf.seek(0)
      self.sock.sendto(buf.read(), (self.udp_ip, self.cam_port))

  def handle_bno_write(self):
    while True:
      eul = self.sensor.euler
      gyro = self.sensor.gyro
      acce = self.sensor.linear_acceleration
      bno_packet = eul + gyro + acce + (time.time()-self.t0,)
      bno_bytes = struct.pack("<10d", *bno_packet)
      self.sock.sendto(bno_bytes, (self.udp_ip, self.bno_port))
      #time.sleep(1./self.hz)

# setup streamer
if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--cam_only', dest='cam', action='store_true')
  parser.add_argument('--imu_only', dest='imu', action='store_true')

  args = parser.parse_args()
  udp_ip = '192.168.3.100'
  stream_out = ThreadedStream(udp_ip, only_cam=args.cam, only_imu=args.imu)

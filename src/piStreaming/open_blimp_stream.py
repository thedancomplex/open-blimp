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
  from io import BytesIO

except Exception as e: print("Camera import error:", e)

# BNO specific libraries
try:
 import serial
 import adafruit_bno055

except Exception as e: print("BNO055 import error:", e)

# VL53L0X specific libraries
try:
 import board
 import busio
 import adafruit_vl53l0x

except Exception as e: print("VL53L0X import error:", e)

class ThreadedStream:
  def __init__(self, udp_ip, udp_ports=(8485, 8486, 8487), uart_port="/dev/serial0", res=(128, 128), quality=10, fps=5, hz=30):

    # setup UDP socket
    print("Streaming to", udp_ip)
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.cam_port = udp_ports[0]
    self.bno_port = udp_ports[1]
    self.dist_port = udp_ports[2]
    self.udp_ip = udp_ip
    self.quality = quality
    self.t0 = time.time()
    self.hz = hz

    try:
      # setup camera
      self.cam = PiCamera()
      self.cam.resolution = res
      self.cam.framerate = fps
      self.cam.shutter_speed = 100
      self.cam.iso = 20

      # setup camera thread
      self.cam_thread = threading.Thread(target=self.handle_cam_write, args=())
      self.cam_thread.start()
      print("Camera successfully registered")

    except Exception as e:
      print("Camera setup skipped:", e)

    try:
      # setup UART
      uart = serial.Serial(uart_port, 115200, timeout=5)
      self.sensor = adafruit_bno055.BNO055_UART(uart)
      self.sensor.mode = 11

      # setup bno thread
      self.bno_thread = threading.Thread(target=self.handle_bno_write, args=())
      self.bno_thread.start()
      print("BNO successfully registered")

    except Exception as e:
      print("BNO setup skipped:", e)

    try:
      #setup I2C
      i2c = busio.I2C(board.SCL, board.SDA)
      self.distance = adafruit_vl53l0x.VL53L0X(i2c)

      self.dist_thread = threading.Thread(target=self.handle_dist_write, args=())
      self.dist_thread.start()
      print("Distance sensor successfully registered")

    except Exception as e:
      print("Distance sensor setup skipped:", e)

  def handle_cam_write(self):
    buf = BytesIO()
    for frame in self.cam.capture_continuous(buf, format="jpeg", quality=self.quality, use_video_port=True):
      buf.seek(0)
      self.sock.sendto(buf.read(), (self.udp_ip, self.cam_port))

      # delete buffer
      buf.seek(0)
      buf.truncate(0)

  def handle_bno_write(self):
    while True:
      quat = self.sensor.quaternion
      gyro = self.sensor.gyro
      bno_packet = quat + gyro
      bno_bytes = struct.pack("<7d", *bno_packet)
      self.sock.sendto(bno_bytes, (self.udp_ip, self.bno_port))

  def handle_dist_write(self):
    while True:
      distance = self.distance.range
      dist_packet = (distance,)
      dist_bytes = struct.pack("<1d", *dist_packet)
      self.sock.sendto(dist_bytes, (self.udp_ip, self.dist_port))
      time.sleep(0.1)

# setup streamer
if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--dest_ip', dest='udp_ip', default="localhost",
                      help="IP of the server-side computer for streaming")

  args = parser.parse_args()
  stream_out = ThreadedStream(udp_ip, only_cam=args.cam, only_imu=args.imu, only_dist=args.dist)

#!/usr/bin/env/python3
# default python libraries
import argparse
import multiprocessing as mp
import socket
import struct
import signal
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

class MultiStream:
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

        self.cam_init = True
        self.bno_init = True
        self.dist_init = True

        try:
            # setup camera
            self.cam = PiCamera()
            self.cam.resolution = res
            self.cam.framerate = fps
            self.cam.shutter_speed = 100
            self.cam.iso = 20
            print("Camera successfully registered")

        except Exception as e:
            print("Camera setup skipped:", e)
            self.cam_init = False

        try:
            # setup UART
            uart = serial.Serial(uart_port, 115200, timeout=5)
            self.sensor = adafruit_bno055.BNO055_UART(uart)
            self.sensor.mode = 11
            print("BNO successfully registered")

        except Exception as e:
            print("BNO setup skipped:", e)
            self.bno_init = False

        try:
            #setup I2C
            i2c = busio.I2C(board.SCL, board.SDA)
            self.distance = adafruit_vl53l0x.VL53L0X(i2c)
            print("Distance sensor successfully registered")

        except Exception as e:
            print("Distance sensor setup skipped:", e)
            self.dist_init = False
          
    
    def run(self):
        # start the camera process
        if self.cam_init:
            self.pcam = mp.Process(target=self.handle_cam_write, args=())
            self.pcam.start()
            
        # start the BNO process
        if self.bno_init:
            self.pbno = mp.Process(target=self.handle_bno_write, args=())
            self.pbno.start()
            
        # start the IR laser process
        if self.dist_init:
            self.pdist = mp.Process(target=self.handle_dist_write, args=())
            self.pdist.start()
            
    def stop(self):
        # stop the camera process
        if self.cam_init:
            self.pcam.terminate()
            self.pcam.join()
            
        # stop the BNO process
        if self.bno_init:
            self.pbno.terminate()
            self.pbno.join()
            
        # stop the IR laser process
        if self.dist_init:
            self.pdist.terminate()
            self.pdist.join()
    
            
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
          
          
# setup streamer
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--udp_ip', dest='udp_ip', default="localhost",
                        help="IP of the server-side computer for streaming")
                        
    args = parser.parse_args()
    stream_out = MultiStream(args.udp_ip)
    stream_out.run()
    
    # end all subprocesses if ctrl-C is caught
    def handler(signum, frame):
        stream_out.stop()

    signal.signal(signal.SIGINT, handler)        
    while True:
        time.sleep(0.1)
        

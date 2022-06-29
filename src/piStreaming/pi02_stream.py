#!/usr/bin/env/python3
# default python libraries
import argparse
import multiprocessing as mp
import socket
import struct
import signal
import sys
import time


class MultiStream:
    def __init__(self, udp_ip, udp_ports=(8485, 8486, 8487)):
        # setup UDP socket
        print("Streaming to", udp_ip)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_ip = udp_ip
        self.cam_port = udp_ports[0]
        self.bno_port = udp_ports[1]
        self.dist_port = udp_ports[2]
        self.t0 = time.time()

        self.cam_running = False
        self.bno_running = False
        self.vl53_running = False

        # store camera params
        self.res = (128, 128)
        self.fps = 20
        self.shutter_speed = 100
        self.quality = 10

    def __exit__(self):
        print("Cleaning up...")
        self.stop()

    def run(self):
        # start the camera process
        if not self.cam_running:
            self.pcam = mp.Process(target=self.handle_cam_write, args=())
            self.pcam.start()
            self.cam_running = True

        # start the BNO process
        if not self.bno_running:
            self.pbno = mp.Process(target=self.handle_bno_write, args=())
            self.pbno.start()
            self.bno_running = True

        # start the IR laser process
        if not self.vl53_running:
            self.pdist = mp.Process(target=self.handle_dist_write, args=())
            self.pdist.start()
            self.vl53_running = True

    def stop(self):
        # stop the camera process
        if self.cam_running:
            self.pcam.terminate()
            self.pcam.join()
            self.cam_running = False

        # stop the BNO process
        if self.bno_running:
            self.pbno.terminate()
            self.pbno.join()
            self.bno_running = False

        # stop the IR laser process
        if self.vl53_running:
            self.pdist.terminate()
            self.pdist.join()
            self.dist_running = False

    def handle_cam_write(self):
        try:
            # import necessary libs
            from picamera import PiCamera
            from io import BytesIO

            # setup the socket
            sock = socket.socket()

            connected = False
            while not connected:
                try:
                    sock.connect((self.udp_ip, self.cam_port))
                    connected = True

                except Exception as e:
                    print("Camera server still trying to connect...")
                    time.sleep(1)

            self.cam_connection = sock.makefile('wb')

            # setup the camera
            cam = PiCamera()
            cam.resolution = self.res
            cam.framerate = self.fps
            cam.shutter_speed = self.shutter_speed
            print("Camera successfully registered")

            # start the camera stream
            buf = BytesIO()
            for frame in cam.capture_continuous(buf, format="jpeg", quality=self.quality, use_video_port=True):
                # write the length of the capture
                buf_len = struct.pack('<L', buf.tell())
                self.cam_connection.write(buf_len)
                self.cam_connection.flush()

                # write the image data
                buf.seek(0)
                self.cam_connection.write(buf.read())

                # delete buffer
                buf.seek(0)
                buf.truncate()

        except Exception as e:
            print("Camera failed: ", e)
            self.cam_running = False

    def handle_bno_write(self):
        try:
            # import necessary libs
            import serial
            import adafruit_bno055

            # setup the socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            # setup the bno055
            uart = serial.Serial("/dev/serial0", 115200, timeout=5)
            sensor = adafruit_bno055.BNO055_UART(uart)
            sensor.mode = 11
            print("BNO055 successfully registered")

            # start the bno055 stream
            while True:
                quat = sensor.quaternion
                gyro = sensor.gyro
                acc  = sensor.acceleration
                bno_packet = quat + gyro + acc
                bno_bytes = struct.pack("<10d", *bno_packet)
                sock.sendto(bno_bytes, (self.udp_ip, self.bno_port))

        except Exception as e:
            print("BNO failed: ", e)
            self.bno_running = False

    def handle_dist_write(self):
        try:
            # import necessary libs
            import board
            import adafruit_vl53l1x

            # setup the socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            # setup the VL53L1X
            i2c = board.I2C()
            vl53 = adafruit_vl53l1x.VL53L1X(i2c)
            vl53.distance_mode = 2
            vl53.timing_budget = 100
            vl53.start_ranging()
            print("VL53L1X successfully registered")

            # start VL53L1X stream
            while True:
                if vl53.data_ready:
                    distance = vl53.distance
                    vl53.clear_interrupt()

                    dist_packet = (distance,)
                    dist_bytes = struct.pack("<1d", *dist_packet)
                    sock.sendto(dist_bytes, (self.udp_ip, self.dist_port))
                    time.sleep(0.05)

        except Exception as e:
            print("VL53 failed: ", e)
            self.vl53_running = False

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
        sys.exit(0)

    signal.signal(signal.SIGINT, handler)

    # check for sensor failures
    while True:
        time.sleep(0.5)
        if not stream_out.cam_running:
            stream_out.pcam.terminate()
            stream_out.pcam.join()
            print("Camera process ended")

        if not stream_out.bno_running:
            stream_out.pbno.terminate()
            stream_out.pbno.join()
            print("BNO055 process ended")

        if not stream_out.vl53_running:
            stream_out.pdist.terminate()
            stream_out.pdist.join()
            print("VL53L1X process ended")

        if not stream_out.cam_running and \
           not stream_out.bno_running and \
           not stream_out.vl53_running:
               break

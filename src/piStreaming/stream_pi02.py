#!/usr/bin/env/python3
# default python libraries
import multiprocessing as mp
import socket
import struct
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

        # store camera params
        self.res = (360, 240)
        self.fps = 20
        self.quality = 15

        # setup the processes
        self.pcam = mp.Process(target=self.handle_cam_write, args=())
        self.pbno = mp.Process(target=self.handle_bno_write, args=())
        self.pdist = mp.Process(target=self.handle_dist_write, args=())

    def __exit__(self):
        self.stop()

    def run(self):
        # start the camera process
        if not self.pcam.is_alive():
            self.pcam.start()

        # start the BNO process
        if not self.pbno.is_alive():
            self.pbno.start()

        # start the IR laser process
        if not self.pdist.is_alive():
            self.pdist.start()

    def stop(self):
        # stop the camera process
        if self.cam_running:
            self.pcam.terminate()
            self.pcam.join()

        # stop the BNO process
        if self.bno_running:
            self.pbno.terminate()
            self.pbno.join()

        # stop the IR laser process
        if self.vl53_running:
            self.pdist.terminate()
            self.pdist.join()

    def handle_cam_write(self):
        #import necessary libs
        from picamera import PiCamera
        from io import BytesIO

        # setup the camera
        cam = PiCamera()
        cam.resolution = self.res
        cam.framerate = self.fps
        cam.exposure_mode = "sports"
        print("Camera successfully registered")

        while True:
            try:
                # setup the socket
                sock = socket.socket()

                # connect to the server
                connected = False
                while not connected:
                    try:
                        sock.connect((self.udp_ip, self.cam_port))
                        connected = True

                    except Exception as e:
                        time.sleep(0.1)

                print("Camera server connected!")
                self.cam_connection = sock.makefile('wb')

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
                print("Camera server disconnected!")

    def handle_bno_write(self):
        try:
            # import necessary libs
            from Adafruit_BNO055 import BNO055

            # setup the socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            # setup the bno055
            sensor = BNO055.BNO055(serial_port='/dev/serial0', rst=18)
            sensor.set_mode(8) # 8- IMU mode, 12 - NDOF mode
            print("BNO055 successfully registered")

            # start the bno055 stream
            while True:
                quat = sensor.read_quaternion()
                gyro = sensor.read_gyroscope()
                acc  = sensor.read_linear_acceleration()
                bno_packet = quat + gyro + acc
                bno_bytes = struct.pack("<10d", *bno_packet)
                sock.sendto(bno_bytes, (self.udp_ip, self.bno_port))

        except Exception as e:
            print("BNO failed: ", e)

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
            vl53.timing_budget = 50
            vl53.start_ranging()
            print("VL53L1X successfully registered")

            # start VL53L1X stream
            while True:
                if vl53.data_ready:
                    distance = vl53.distance
                    vl53.clear_interrupt()

                    if distance is not None:
                        dist_packet = (distance,)
                        dist_bytes = struct.pack("<1d", *dist_packet)
                        sock.sendto(dist_bytes, (self.udp_ip, self.dist_port))

                time.sleep(0.05)

        except Exception as e:
            print("VL53 failed: ", e)

# setup streamer
if __name__ == "__main__":
    import argparse
    import signal
    import sys

    parser = argparse.ArgumentParser()
    parser.add_argument('--udp_ip', dest='udp_ip', default="localhost",
                        help="IP of the server-side computer for streaming")

    args = parser.parse_args()
    S = MultiStream(args.udp_ip)
    S.run()

    # end all subprocesses if ctrl-C is caught
    def handler(signum, frame):
        stream_out.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, handler)

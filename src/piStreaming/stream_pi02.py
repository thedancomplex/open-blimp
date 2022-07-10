#!/usr/bin/env/python3
import multiprocessing as mp
import numpy as np
import signal
import socket
import struct
import time
import traceback

from multiprocessing import shared_memory as sm


class MultiStream:
    def __init__(self):
        # setup the process names
        self.pcam = None
        self.pbno = None
        self.pdis = None

        # setup the running flag
        self.sh_flag = sm.SharedMemory(create=True, size=1)
        self.flag = np.ndarray(1, dtype=np.bool_, buffer=self.sh_flag.buf)
        self.flag[0] = False
                
        # setup the running flag (to be stopped in ctrl-c event)
        self.run = True

        signal.signal(signal.SIGINT, self.handler)

    def handler(self, signum, frame):
        traceback.print_exc()
        self.run = False

    def run(self):
        # wait for a start signal
        while self.run:
            # setup the socket for parameter reading (on port 8485 by default)
            sock = socket.socket()
            sock.settimeout(0.5)
            sock.bind(('0.0.0.0', 8485))
            sock.listen(0)

            # wait for a request
            connected = False
            print("Waiting for connection...")
            while not connected and self.run:
                try: con, connected = sock.accept()[0].makefile('rb'), True
                except: pass
                
            # read in the parameters to start
            data = []
            if connected:
                try: data = con.read()
                except: pass

            # if stop signal is read, set flag and stop
            nbytes = len(data)            
            if nbytes == 4:
                # parse data
                msg = str(data)[2:][:-1]
            
                # check if correct message
                if msg == 'STOP': self.flag[0] = False
                print("Stop heard! Ending streams")

                # wait for processes to finish before resetting
                self.pcam.join()
                self.pbno.join()
                self.pdis.join()

            if nbytes == 11:
                # parse data
                # - 1 byte id_num, 4 bytes IP, 4 bytes img size, 1 byte fps, 1 byte quality 
                id_data = data[0]
                ip_data = data[1:5]
                res_data = data[5:9]
                fps_data = data[9]
                qual_data = data[10]

                id_num = struct.unpack("<1B", id_data)
                ip = struct.unpack("<4B", ip_data)
                ip = ".".join(map(str, ip))
                res = struct.unpack("<2H", res_data)
                fps = struct.unpack("<1B", fps_data)
                qual = struct.unpack("<1B", qual_data)

                # start streams
                self.flag[0] = True

                # define ports based on id_num
                c_port = 1024 + 3*(id_num-1)
                b_port = 1025 + 3*(id_num-1)
                d_port = 1026 + 3*(id_num-1)

                # setup the processes
                args = (ip, c_port, res, fps, qual, self.sh_flag.name)
                self.pcam = mp.Process(target=self.handle_cam, args=args)
                self.pbno = mp.Process(target=self.handle_bno, args=(ip, b_port, fname))
                self.pdis = mp.Process(target=self.handle_dis, args=(ip, d_port, fname))

                # start the processes
                self.t0 = time.time()
                self.pcam.start()
                self.pbno.start()
                self.pdis.start()
                print("Connected to", ip)

            # close the socket in preparation for a new request
            sock.close()

        # shutdown operations
        self.flag[0] = False
        if self.pcam is not None:
            self.pcam.join()
            
        if self.pbno is not None:
            self.pbno.join()

        if self.pdis is not None:
            self.pdis.join()

        self.flag[0].close()
        self.flag[0].unlink()


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

    def handle_cam(self, ip, port, res, fps, q, fname):
        # see if camera libs are installed
        try:
            from picamera import PiCamera
            from io import BytesIO

        except:
            print("Camera library import failed! Ignoring camera stream") 
            return
        
        # setup shared memory flag
        sh_flag = sm.SharedMemory(fname)
        flag = np.ndarray(1, dtype=np.bool_, buffer=sh_flag.buf)

        # setup the camera
        cam = PiCamera()
        cam.resolution = res
        cam.framerate = fps
        cam.exposure_mode = "sports"

        # connect to the server
        sock = socket.socket()
        sock.settimeout(0.5)
        connected = False
        while not connected:
            try:
                sock.connect((ip, port))
                connected = True

            except: pass

        print("Camera successfully registered")
        cam_connection = sock.makefile('wb')

        # start the camera stream
        buf = BytesIO()
        t0 = time.time()
        for frame in cam.capture_continuous(buf, format="jpeg", quality=q, use_video_port=True):

            # break if run flag is off
            if not flag[0]: break

            # get the current time
            tframe = time.time() - self.t0

            # write the length of the capture
            buf_len = struct.pack('<L', buf.tell())
            cam_connection.write(buf_len)
            cam_connection.flush()

            # write the image data
            buf.seek(0)
            cam_connection.write(buf.read())
            cam_connection.flush()

            # write the current time
            tbuf = struct.pack('<d', tframe)
            cam_connection.write(tbuf)

            # delete buffer
            buf.seek(0)
            buf.truncate()

        # cleanup
        sh_flag.close()
        cam_connection.close()
        cam.close()
        
    def handle_bno_write(self, ip, port, fname):
        # see if BNO055 libs are installed
        try:
            from Adafruit_BNO055 import BNO055

        except:
            print("BNO055 library import failed! Ignoring imu stream") 
            return
        
        # setup shared memory flag
        sh_flag = sm.SharedMemory(fname)
        flag = np.ndarray(1, dtype=np.bool_, buffer=sh_flag.buf)

        # setup the socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # setup the bno055
        sensor = BNO055.BNO055(serial_port='/dev/serial0', rst=18)
        sensor.set_mode(8) # 8- IMU mode, 12 - NDOF mode
        print("BNO055 successfully registered")

        # start the bno055 stream
        while flag[0]:
            quat = sensor.read_quaternion()
            gyro = sensor.read_gyroscope()
            acc  = sensor.read_linear_acceleration()
            tframe = time.time() - self.t0

            bno_packet = quat + gyro + acc + (tframe,)
            bno_bytes = struct.pack("<11d", *bno_packet)
            sock.sendto(bno_bytes, (self.udp_ip, self.bno_port))

        # cleanup
        sh_flag.close()
        sock.close()

    def handle_dist_write(self):
        # see if VL53L1X libs are installed
        try:
            import board
            import adafruit_vl53l1x

        except:
            print("VL53L1x library import failed! Ignoring distance stream") 
            return

        # setup shared memory flag
        sh_flag = sm.SharedMemory(fname)
        flag = np.ndarray(1, dtype=np.bool_, buffer=sh_flag.buf)

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
        while flag[0]:
            if vl53.data_ready:
                distance = vl53.distance
                vl53.clear_interrupt()

                if distance is not None:
                    tframe = time.time() - self.t0
                    dist_packet = (distance, tframe)
                    dist_bytes = struct.pack("<2d", *dist_packet)
                    sock.sendto(dist_bytes, (self.udp_ip, self.dist_port))

            time.sleep(0.02)

        # cleanup
        sh_flag.close()
        sock.close()


# setup streamer
if __name__ == "__main__":
    S = MultiStream()
    S.run()

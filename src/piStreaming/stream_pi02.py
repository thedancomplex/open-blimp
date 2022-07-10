#!/usr/bin/env/python3
import multiprocessing as mp
import numpy as np
import signal
import socket
import struct
import time

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
        self.running = True

        signal.signal(signal.SIGINT, self.handler)

    def handler(self, signum, frame):
        self.running = False

    def run(self):
        sock = socket.socket()
        sock.bind(('0.0.0.0', 8485))
        sock.listen(0)
        sock.settimeout(2)

        # wait for a start signal
        while self.running:
            # setup the socket for listening

            # wait for a request
            connected = False
            print("Waiting for connection...")
            while not connected and self.running:
                print("Waiting...")
                try: 
                    con = sock.accept()[0].makefile('rb')
                    connected = True

                except: pass
            
            print("Connected!")
            # read in the parameters to start
            data = []
            if connected:
                try: data = con.read(11)
                except: pass

            # if stop signal is read, set flag and stop
            if len(data) == 11:
                # parse data
                msg = str(data)[2:][:-1]
            
                # check if correct message
                if msg == 'STOP0000000': 
                    self.flag[0] = False
                    print("Stop heard! Ending streams")

                    # wait for processes to finish before resetting
                    if self.pcam is not None: self.pcam.join()
                    if self.pbno is not None: self.pbno.join()
                    if self.pdis is not None: self.pdis.join()


                # - 1 byte id_num, 4 bytes IP, 4 bytes img size, 1 byte fps, 1 byte quality 
                id_num = data[0]
                ip = struct.unpack("<4B", data[1:5])
                ip = ".".join(map(str, ip))
                res = struct.unpack("<2H", data[5:9])
                fps = data[9]
                qual = data[10]

                # start streams
                self.flag[0] = True

                # define ports based on id_num
                c_port = 1024 + 3*(id_num-1)
                b_port = 1025 + 3*(id_num-1)
                d_port = 1026 + 3*(id_num-1)

                # setup the processes
                fname = self.sh_flag.name
                args = (ip, c_port, res, fps, qual, fname)
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
            #sock.shutdown()
            #sock.close()

        # shutdown operations
        self.flag[0] = False
        if self.pcam is not None:
            self.pcam.join()
            
        if self.pbno is not None:
            self.pbno.join()

        if self.pdis is not None:
            self.pdis.join()

        self.sh_flag.close()
        self.sh_flag.unlink()

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
        sock.settimeout(2)
        connected = False
        while not connected and flag[0]:
            try:
                sock.connect((ip, port))
                connected = True

            except: pass

        # return prematurely if stopped early
        if not connected: 
            sh_flag.close()
            cam.close()
            return        
               
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

            try:
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
                cam_connection.flush()

            except: 
                # if the cam connection fails for some reason
                print("Broken image pipe!")
                sh_flag.close()
                #cam.close()
                return
                
            # delete buffer
            buf.seek(0)
            buf.truncate()

        # cleanup
        cam_connection.close()
        sh_flag.close()
        cam.close()

    def handle_bno(self, ip, port, fname):
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
            sock.sendto(bno_bytes, (ip, port))

        # cleanup
        sh_flag.close()
        sock.close()

    def handle_dis(self, ip, port, fname):
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
                    sock.sendto(dist_bytes, (ip, port))

            time.sleep(0.02)

        # cleanup
        sh_flag.close()
        sock.close()


# setup streamer
if __name__ == "__main__":
    S = MultiStream()
    S.run()

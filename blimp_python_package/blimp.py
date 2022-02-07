import time
import serial
import struct
import numpy as np
from pid import PID
from utils.mixer import *
from scipy.spatial.transform import Rotation as R
from pi_zero_w_streaming.threaded_stream_rcv import ThreadedPiStream

class Blimp:
    def __init__(self, port, stream_ip, logger=True):
        self.stream_ip = stream_ip
        self.logger = logger
  
        # construct serial port
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = 921600
        self.ser.write_timeout = 0
        self.ser.open()

        # construct lowest-level PID controllers
        k_rp = np.array([1., 0.01, 0.5])
        k_yw = np.array([1., 0.01, 0.5])
        self.pid_roll = PID(k_rp, angle=True)
        self.pid_pitch = PID(k_rp, angle=True)
        self.pid_yaw = PID(k_yw, angle=True)

        # construct velocity PID controllers
        k_vxy = np.array([1., 0.01, 0.5])        
        self.pid_vx = PID(k_vxy, windup=1.0)
        self.pid_vy = PID(k_vxy, windup=1.0)

        # construct position PID controllers
        k_pxy = np.array([1., 0.01, 0.5])
        k_pz = np.array([1., 0.01, 0.5])
        self.pid_px = PID(k_pxy, windup=0.5)
        self.pid_py = PID(k_pxy, windup=0.5)
        self.pid_pz = PID(k_pz, windup=0.5)

        # setup logger information
        self.x = np.zeros(13); self.x[9] = 1.
        self.x_stamp = None
        self.I_stamp = None
        if self.logger:
            self.idx = 0
            self.uh = np.empty((6,1000))
            self.xh = np.empty((13,1000))
            self.Ih = np.empty((360,360,3,1000))

        # setup input buffer to be sent out
        self.u = np.zeros(6)

        # setup sensing tools
        #self.pi = ThreadedPiStream(8485, 8486)

    def poll_pi(self):
        # call to update the state data
        # - alternatively can be re-written to be done in the background
        I_stamp, bno_stamp, I, bno = self.pi.get_pair()
        self.x[6:10] = bno[0]
        self.x[10:] = bno[1]
        self.I = I
        self.x_stamp = bno_stamp
        self.I_stamp = I_stamp

    def get_state(self):
        return self.x.copy()

    def get_image(self):
        return self.I.copy()

    def set_wp(self, goal):
        # compute the full three loop position control input
        des_vx = self.pid_px.input(self.x[0], goal[0])
        des_vy = self.pid_py.input(self.x[1], goal[1])

        # compute the velocity input
        self.set_vel([des_vx, des_vy])

    def set_vel(self, vel):
        # compute the double loop velocity control input
        des_r = self.pid_vx.input(self.x[3], vel[0])
        des_p = self.pid_vy.input(self.x[4], vel[1])

        # compute the angle input
        self.set_ang([des_r, des_p])

    def set_ang(self, ang):
        # compute the single loop angle control input
        [r, p, yw] = R.from_quat(self.x[6:10]).as_euler('xyz')

        # - inputs
        self.u[0] = self.pid_roll.input(r, ang[0])
        self.u[1] = self.pid_pitch.input(p, ang[1])

    def set_heading(self, yw_des):
        # compute the single loop angle control input for heading
        [_, _, yw] = R.from_quat(self.x[6:10]).as_euler('xyz')
        
        # - inputs
        self.u[5] = self.pid_yaw.input(yw, yw_des)

    def set_alt(self, alt):
        # compute the single loop altitude control input
        self.u[2] = self.pid_pz.input(self.x[2], alt)

    def step(self, cmd=None):
        if cmd is not None:
            self.u = cmd

        # mix commands to get motor inputs
        tau = actuation_vector_saturation(self.u)
        f = mixer_positive(tau)
        duty_cycles = thrust2dutyCycle(f)
        duty_cycles = duty_cycle_saturation(duty_cycles)
        duty_cycles = direction_and_order_mapping(duty_cycles)
        msg = convertCMD(duty_cycles)

        msgb = b''
        for i in msg:
            msgb += struct.pack('!B', int(i))

        self.ser.write(msgb)
        
        # log data (if requested)
        if self.logger:
            self.uh[:,self.idx] = self.u
            self.xh[:,self.idx] = self.x
            self.Ih[:,:,:,self.idx] = self.I
            self.idx += 1

            if self.idx > self.uh.shape[1]:
                self.uh = np.concatenate(self.uh, np.empty((6,1000)), axis=1)
                self.xh = np.concatenate(self.xh, np.empty((13,1000)), axis=1)
                self.Ih = np.concatenate(self.Ih, np.empty((360,360,3,1000)), axis=3)

    def save(self, fname):
        # save historical data if logger flag is active
        pass

import os
import time
import serial
import struct
import numpy as np
from numpy import pi
from pid import PID
from utils.mixer import *
from scipy.spatial.transform import Rotation as R
from pi_zero_w_streaming.threaded_stream_rcv import ThreadedPiStream
from utils.NatNetBlimp import Natnet_blimp

class Blimp:
    def __init__(self, port, my_ip, pi_ip=None, opti_ip=None, logger=True):
        self.port = port
        self.my_ip = my_ip
        self.pi_ip = pi_ip
        self.opti_ip = opti_ip

        self.logger = logger
  
        # construct serial port
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = 921600
        self.ser.write_timeout = 0
        self.ser.open()

        # construct lowest-level PID controllers
        k_r = np.array([-0.17, 0.001, 2.13])
        k_p = np.array([-0.17, 0.001, 2.11])
        k_yw = np.array([0.0006, 0.0, 0.001])
        self.pid_roll = PID(k_r, angle=True)
        self.pid_pitch = PID(k_p, angle=True)
        self.pid_yaw = PID(k_yw, angle=True)

        # construct velocity PID controllers
        k_vxy = np.array([0.5, 0., 0.])
        self.pid_vx = PID(k_vxy, windup=1.0)
        self.pid_vy = PID(k_vxy, windup=1.0)

        # construct position PID controllers
        k_pxy = np.array([0., 0., 0.])
        k_pz = np.array([0.19, 0.001, 0.17])
        self.pid_px = PID(k_pxy, windup=0.5)
        self.pid_py = PID(k_pxy, windup=0.5)
        self.pid_pz = PID(k_pz, windup=0.5)

        # setup logger information
        self.x = np.zeros(13); self.x[9] = 1.
        self.x_stamp = None
        self.z_stamp = None
        self.I_stamp = None
        if self.logger:
            self.idx = 0
            self.uh = []
            self.xh = []
            self.Ih = []

        # setup input buffer to be sent out
        self.u = np.zeros(6)

        if opti_ip:
            # setup natnet client for optitrack system
            self.NatNet = Natnet_blimp(my_ip, opti_ip)
            self.runNatNet()

        # setup sensing tools
        if pi_ip:
            self.pi = ThreadedPiStream(8485, 8486, 8487)

        # set initial zeros
        self.rot0 = [0., 0., 0.]

    def poll_image(self):
        # call to update the state data
        # - alternatively can be re-written to be done in the background
        self.I_stamp, self.I = self.pi.get_image()

    def poll_bno(self):
        # call to update the state data
        # - alternatively can be re-written to be done in the background
        self.x_stamp, bno = self.pi.get_bno()
        self.x[6:10] = bno[0]
        self.x[10:] = bno[1]

    def poll_dist(self):
        # call to update the state data
        # - alternatively can be re-written to be done in the background
        self.z_stamp, z = self.pi.get_dist()
        z = min(z, 1.25)
        
        # correct distance using angle
        [r, p, yw] = self.quat_to_eul(self.x[6:10])
        self.x[2] = z*np.cos(r)*np.cos(p)

    def quat_to_eul(self, q):
        [yw, p, r] = R.from_quat(q).as_euler('xyz')

        # transform to blimp coordinate system
        r += pi
        r -= 2*pi*(r > pi)

        return -r, -p, yw

    def zero_xy_rot(self):
        self.poll_bno()
        [r, p, _] = self.quat_to_eul(self.x[6:10])
        self.rot0[:2] = [r, p]
        
    def zero_z_rot(self):
        self.poll_bno()
        self.rot0[2] = self.quat_to_eul(self.x[6:10])[2]

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
        des_p = self.pid_vx.input(self.x[3], vel[0])
        des_r = self.pid_vy.input(self.x[4], vel[1])

        # compute the angle input
        self.set_ang([des_p, des_r])

    def set_ang(self, ang=[0,0]):
        # compute the single loop angle control input
        [r, p, _] = self.quat_to_eul(self.x[6:10])

        # adjust to 0-settings
        r -= self.rot0[0]
        p -= self.rot0[1]

        # get roll and pitch velocities
        vr = -self.x[10]
        vp = -self.x[11]
        
        # - inputs
        phat = np.array([p, vp])
        pdes = np.array([ang[1], 0.])
        rhat = np.array([r, vr])
        rdes = np.array([ang[0], 0.])
        self.u[0] = -self.pid_pitch.input(phat, pdes, given_velocity=True)
        self.u[1] = self.pid_roll.input(rhat, rdes, given_velocity=True)

    def set_heading(self, yw_des=0.):
        # compute the single loop angle control input for heading
        [_, _, yw] = self.quat_to_eul(self.x[6:10])

        # adjust to 0-settings
        yw -= self.rot0[2]
        
        # - inputs
        self.u[5] = self.pid_yaw.input(yw, yw_des)

    def set_alt(self, alt, positive_only=False):
        # compute the single loop altitude control input
        self.u[2] = -self.pid_pz.input(self.x[2], alt)
        if positive_only:
          self.u[2] = min(self.u[2], 0.)

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
            self.idx += 1
            self.uh.append(self.u.copy())
            self.xh.append(self.x.copy())
            if self.pi_ip:
                self.Ih.append(self.I.copy())

    def save(self, save_dir):
        # save historical data if logger flag is active
        tsave = str(np.datetime64('now'))
        usave = "inputs_"+tsave+".npy"
        xsave = "states_"+tsave+".npy"

        # concatenate to 2D arrays
        self.uh = np.stack(self.uh).T
        self.xh = np.stack(self.xh).T
        self.Ih = np.stack(self.Ih)
        
        np.save(os.path.join(save_dir, usave), self.uh[:,:self.idx])
        np.save(os.path.join(save_dir, xsave), self.xh[:,:self.idx])
        if self.pi_ip:
            Isave = "images_"+tsave+".npy"
            np.save(os.path.join(save_dir, Isave), self.Ih[:,:,:,:self.idx])

    def receive_rigid_body_frame(self, new_id, position, rotation ):
        # callback funtion for optitrack system to get pose and position
        self.x[:3] = position
        self.x[6:10] = rotation
        pass

    def runNatNet(self):
        # start Optitrack
        self.NatNet.streaming_client.rigid_body_listener = self.receive_rigid_body_frame
        self.NatNet.streaming_client.run()
    
    def stopNatNet(self):
        # stop Optitrack
        self.NatNet.streaming_client.shutdown()

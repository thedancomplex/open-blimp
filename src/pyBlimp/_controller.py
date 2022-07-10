import multiprocessing as mp
import numpy as np
import os
import signal
import struct
import time

from multiprocessing import shared_memory as sm
from numpy import pi
from piStreaming.rcv_pi02 import MultiRcv
from pyBlimp.mixer import *
from pyBlimp.pid import PID
from pyBlimp.utils import *
from scipy.spatial.transform import Rotation as R


def handle_controller(ser, cfg, locks, names, logger):
    """ private controller handler spawned by a parent Blimp object
        - this handle should NOT be used by an application
        - only spawned and controlled by a parent Blimp object    
    """
    C = _Controller(ser, cfg, locks, names, logger)
    C.run()


class _Controller:
    """ private controller that runs on a spawned process to control the blimp
        - this controller should NOT be used by an application
        - only spawned and controlled by a parent Blimp object    
    """

    def __init__(self, ser, cfg, locks, names, logger=False):
        im_sz = (cfg['im_cols'], cfg['im_rows'], 3)
        self.id_num = cfg['id_num']
        self.logger = logger
        self.hz = 100.

        # store serial data
        self.ser = ser[0]
        self.lock_u = ser[1]
        
        # store locks and setup shared memory
        self.lock_x = locks[0]
        self.sh_x = sm.SharedMemory(names[0])        
        self.sh_x_stamp = sm.SharedMemory(names[1])
        self.x = np.ndarray(10, dtype=np.double, buffer=self.sh_x.buf)
        self.x_stamp = np.ndarray(1, np.double, buffer=self.sh_x_stamp.buf)

        self.lock_I = locks[1]
        self.sh_img = sm.SharedMemory(names[2])        
        self.sh_img_stamp = sm.SharedMemory(names[3])
        self.img = np.ndarray(im_sz, dtype=np.uint8, buffer=self.sh_img.buf)
        self.img_stamp = np.ndarray(1, np.double, buffer=self.sh_img_stamp.buf)

        self.lock_des = locks[2]
        self.sh_des = sm.SharedMemory(names[4])
        self.des = np.ndarray(4, dtype=np.double, buffer=self.sh_des.buf)

        self.lock_rot0 = locks[3]
        self.sh_rot0 = sm.SharedMemory(names[5])
        self.rot0 = np.ndarray(3, dtype=np.double, buffer=self.sh_rot0.buf)

        self.lock_cmd = locks[4]
        self.sh_man = sm.SharedMemory(names[6])
        self.man = np.ndarray(1, dtype=np.bool_, buffer=self.sh_man.buf)
        self.sh_cmd = sm.SharedMemory(names[7])
        self.cmd = np.ndarray(4, dtype=np.double, buffer=self.sh_cmd.buf)        

        self.sh_run = sm.SharedMemory(names[8])
        self.run = np.ndarray(1, dtype=np.bool_, buffer=self.sh_run.buf)

        # setup sensing tools (and sleep to read some data)
        self.pi = MultiRcv(cfg)
        
        # construct lowest-level PID controllers
        self.positive_only = True
        k_r  = np.array([0.0, 0.0, 0.0])
        k_p  = np.array([0.0, 0.0, 0.0])
        k_yw = np.array([0.0, 0.0, 0.0])
        k_pz = np.array([0.0, 0.0, 0.0])
        self.pid_roll  = PID(k_r,  angle=True)
        self.pid_pitch = PID(k_p,  angle=True)
        self.pid_yaw   = PID(k_yw, angle=True)
        self.pid_pz    = PID(k_pz, windup=0.5)

        # setup logger
        if self.logger:
            self.idx = 0
            self.uh = []
            self.xh = []
            self.Ih = []
            self.th = []

        # setup control-c handler
        signal.signal(signal.SIGINT, self.handler)

    def handler(self, signum, frame):
        # - ctrl-c handler to start process cleanup
        self.run[0] = False

    # main loop
    def run(self):
        # - main control loop that runs at a desired frequency
        # - and executes the angle controller at a high rate
        # - and handles the associated shutdown

        # run this loop continuously at desired rate
        while self.run[0]:
            self.poll()
            self.step()
            time.sleep(1./self.hz)

        # shutdown the raspberry pi
        self.pi.shutdown()

        # cleanup shared memory
        self.sh_x.close()
        self.sh_x_stamp.close()
        self.sh_img.close()
        self.sh_img_stamp.close()
        self.sh_des.close()
        self.sh_rot0.close()
        self.sh_man.close()
        self.sh_cmd.close()
        self.sh_run.close()

        # return if logger flag not active        
        if not self.logger: return

        # save data to directory
        tsave = str(np.datetime64('now'))        
        usave = os.path.join(tsave, "inputs.npy")
        xsave = os.path.join(tsave, "states.npy")
        Isave = os.path.join(tsave, "images.npy")
        Ssave = os.path.join(tsave, "stamps.npy")

        # concatenate to 2D arrays
        uh = np.stack(self.uh).T
        xh = np.stack(self.xh).T
        Ih = np.stack(self.Ih)
        sh = np.stack(self.th)

        # check if save_dir exists first
        save_dir = "data"
        spec_dir = os.path.join(save_dir, tsave)
        if not os.path.isdir(save_dir):
            os.mkdir(save_dir)

        # check if specific_dir exists first
        if not os.path.isdir(spec_dir):
            os.mkdir(spec_dir)
                
        # save blimp data
        np.save(os.path.join(save_dir, usave), uh[:,:self.idx])
        np.save(os.path.join(save_dir, xsave), xh[:,:self.idx])
        np.save(os.path.join(save_dir, Isave), Ih[:,:,:,:self.idx])
        np.save(os.path.join(save_dir, Ssave), sh[:self.idx])
        
    # interfacing with pi
    def poll(self):
        # get most recent bno and distance data
        stamp, bno = self.pi.get_bno()
        _, z = self.pi.get_dis()

        if bno is not None:
            # protected read
            self.lock_rot0.acquire()
            rot0 = self.rot0.copy()
            self.lock_rot0.release()
            eul = quat2euler(bno[0])

            # protected write
            self.lock_x.acquire()
            self.x[0]   =  z               # altitude
            self.x[1:4] =  wrap(eul-rot0)  # euler
            self.x[4]   =  bno[1][0]       # vroll
            self.x[5]   = -bno[1][1]       # vpitch
            self.x[6]   = -bno[1][2]       # vyaw
            self.x[7]   =  bno[2][0]       # xacc
            self.x[8]   = -bno[2][1]       # yacc 
            self.x[9]   = -bno[2][2]       # zacc
            self.x_stamp[0] = stamp
            self.lock_x.release()

        # get most recent image
        stamp, I = self.pi.get_image()

        if I is not None:
            # protected write
            self.lock_I.acquire()
            self.img[:,:,:] = I
            self.img_stamp[0] = stamp
            self.lock_I.release()

    def step(self):
        # input to write
        u = np.zeros(6)

        # read necessary data
        x_ = self.x.copy()

        self.lock_des.acquire()
        des = self.des.copy()
        self.lock_des.release()

        # manual flight control
        if self.man[0]:
            self.lock_cmd.acquire()
            u[0] = self.cmd[0]
            u[1] = self.cmd[1]
            u[2] = self.cmd[2]
            u[5] = self.cmd[3]
            self.lock_cmd.release()
            
        # autopilot flight control                    
        else:   
            # get altitude, roll, pitch, yaw and velocities
            z, eul, veul = x_[0], x_[1:4], x_[4:7]

            # - inputs
            r_  = np.array([eul[0], veul[0]])
            p_  = np.array([eul[1], veul[1]])
            yw_ = np.array([eul[2], veul[2]])
            
            u[0] = -self.pid_pitch.input(p_, des[1], given_velocity=True)
            u[1] =  self.pid_roll.input(r_, des[0], given_velocity=True)
            u[2] = -self.pid_pz.input(z, des[3])
            u[5] =  self.pid_yaw.input(yw_, des[2], given_velocity=True)

            if self.positive_only:
                u[2] = min(u[2], 0.)

        # prioritize vertical thrust
        updown_abs = u[2]
        if abs(updown_abs) > 0.1:
            u *= 0.3

        # clip extraneous input
        u[2] = updown_abs
        u = np.clip(u, -0.8, 0.8)

        # mix commands to get motor inputs
        duty_cycles = mix_inputs(u)

        # add unique ID to end
        msg = convertCMD(duty_cycles, self.id_num)

        msgb = b''
        for val in msg:
            msgb += struct.pack('!B', int(val))

        self.ser.write(msgb)
        
        # log data (if requested)
        if self.logger:
            self.lock_I.acquire()
            I = self.img.copy()
            self.lock_I.release()
        
            self.idx += 1
            self.uh.append(u.copy())
            self.xh.append(x_.copy())
            self.Ih.append(I.copy())
            self.th.append(self.x_stamp[0].copy())


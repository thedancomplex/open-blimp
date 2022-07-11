import multiprocessing as mp
import numpy as np
import time

from multiprocessing import shared_memory as sm
from pyBlimp._controller import handle_controller
from pyBlimp.utils import *

class Blimp:
    def __init__(self, ser, cfg, logger=False):
        """ Main interfacing class for a user to control a single blimp
            - ser is a serial object produced from pySerial
            - cfg is a dict containing parameters for starting the blimp
        """
        
        # define incoming image size
        im_sz = (cfg['im_rows'], cfg['im_cols'], 3)

        # setup shared memory state (1-z, 4-quat, 3-euler, 3-linear accel)
        self.lock_x = mp.Lock()
        self.sh_x = sm.SharedMemory(create=True, size=80)
        self.sh_x_stamp = sm.SharedMemory(create=True, size=8)
        self.x = np.ndarray(10, dtype=np.double, buffer=self.sh_x.buf)
        self.x_stamp = np.ndarray(1, np.double, buffer=self.sh_x_stamp.buf)
        self.x[:] = 0.
        self.x_stamp[0] = 0.        

        # setup shared image access
        self.lock_I = mp.Lock()
        self.sh_img = sm.SharedMemory(create=True, size=np.prod(im_sz))        
        self.sh_img_stamp = sm.SharedMemory(create=True, size=8)
        self.img = np.ndarray(im_sz, dtype=np.uint8, buffer=self.sh_img.buf)
        self.img_stamp = np.ndarray(1, np.double, buffer=self.sh_img_stamp.buf)
        self.img[:,:,:] = 0
        self.img_stamp[0] = 0.

        # setup shared desired states (z, roll, pitch, yaw)
        self.lock_des = mp.Lock()
        self.sh_des = sm.SharedMemory(create=True, size=32)
        self.des = np.ndarray(4, dtype=np.double, buffer=self.sh_des.buf)
        self.des[:] = [0., 0., 0., 1.]

        # setup shared rotation zeros
        self.lock_rot0 = mp.Lock()
        self.sh_rot0 = sm.SharedMemory(create=True, size=24)
        self.rot0 = np.ndarray(3, dtype=np.double, buffer=self.sh_rot0.buf)

        # setup shared manual mode and user interface
        self.lock_cmd = mp.Lock()
        self.sh_man = sm.SharedMemory(create=True, size=1)
        self.man = np.ndarray(1, dtype=np.bool_, buffer=self.sh_man.buf)
        self.sh_cmd = sm.SharedMemory(create=True, size=32)
        self.cmd = np.ndarray(4, dtype=np.double, buffer=self.sh_cmd.buf)        
        self.man[0] = True
        self.cmd[:] = np.zeros(4)

        # setup shared process flag
        self.sh_run = sm.SharedMemory(create=True, size=1)
        self.run = np.ndarray(1, dtype=np.bool_, buffer=self.sh_run.buf)
        self.run[0] = True

        # start the controller process
        locks = (self.lock_x, self.lock_I, self.lock_des, self.lock_rot0, self.lock_cmd)
             
        names = (self.sh_x.name, self.sh_x_stamp.name, self.sh_img.name, 
                 self.sh_img_stamp.name, self.sh_des.name, self.sh_rot0.name,
                 self.sh_man.name, self.sh_cmd.name, self.sh_run.name)

        args = (ser, cfg, locks, names, logger)

        self.pcontroller = mp.Process(target=handle_controller, args=args)
        self.pcontroller.start()

        # set the zeros (wait a bit to make sure data is available)
        time.sleep(1.0)
        self.zero_xy_rot()
        self.zero_z_rot()

    # shutdown operation
    def shutdown(self):
        # set shutdown flag
        self.run[0] = False
        self.pcontroller.join()

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

        self.sh_x.unlink()
        self.sh_x_stamp.unlink()
        self.sh_img.unlink()
        self.sh_img_stamp.unlink()
        self.sh_des.unlink()
        self.sh_rot0.unlink()
        self.sh_man.unlink()
        self.sh_cmd.unlink()
        self.sh_run.unlink()

    # user-interface
    def get_image(self):
        # protected read
        self.lock_I.acquire()
        I_ = self.img.copy()
        self.lock_I.release()
        return I_/255.

    def get_state(self):
        """ get the full corrected state
            - returns in local blimp coordinate frame
            - returns (pz, r, p, yw, vr, vp, vyw, ax, ay, az)
        """
        # protected read
        self.lock_x.acquire()
        x_ = self.x.copy()
        self.lock_x.release()    
        return x_

    def get_euler(self):
        """ get the linear acceleration (without gravity)
            - returns in local blimp coordinate frame
            - returns (r, p, yw)
        """
        # protected read
        self.lock_x.acquire()
        x_ = self.x.copy()
        self.lock_x.release()
        return x_[1:4]

    def get_veuler(self):
        """ get the angular velocity
            - returns in local blimp coordinate frame
            - returns (vr, vp, vyw)
        """
        # protected read
        self.lock_x.acquire()
        x_ = self.x.copy()
        self.lock_x.release()
        return x_[5:8]    

    def get_accel(self):
        """ get the linear acceleration (without gravity)
            - returns in local blimp coordinate frame
            - returns (ax, ay, az)
        """
        # protected read
        self.lock_x.acquire()
        x_ = self.x.copy()
        self.lock_x.release()
        return x_[8:]

    def get_alt(self):
        """ get the roll/pitch corrected altitude
            - returns in local blimp coordinate frame
            - uses a write-safe lock to get the altitude
        """
        # protected read
        self.lock_x.acquire()
        x_ = self.x.copy()
        self.lock_x.release()    

        # correct altitude and return
        eul = blimp_coordinates(euler(x_[1:5]), self.rot0.copy())
        return x_[0]*np.cos(eul[0])*np.cos(eul[1])

    def set_des(self, des):
        """ set the desired local frame states for the blimp to track
            - des is a numpy array of dim 4x1 with values as
            - des[0] : desired roll
            - des[1] : desired pitch
            - des[2] : desired yaw
            - des[3] : desired altitude
            
        """
    
        # limit the desired states to appropriate space    
        des[0] = np.clip(des[0], 0.0, 2.5)
        des[1:] = wrap(des[1:])
        
        # engage auto mode (no lock needed)
        self.man[0] = False
        
        # protected write to desired state
        self.lock_des.acquire()
        self.des[:] = des
        self.lock_des.release()

    def set_cmd(self, cmd):
        """ set the manual inputs to apply
            - cmd is a numpy array of dim 4x1 with values as
            - cmd[0] : force along x
            - cmd[1] : force along y
            - cmd[2] : force along z
            - cmd[3] : torque along z
        """

        # engage manual mode (no lock needed)
        self.man[0] = True

        # protected write to manual input
        self.lock_cmd.acquire()
        self.cmd[:] = cmd
        self.lock_cmd.release()        

    # setup functions
    def zero_xy_rot(self):
        """ grabs the current roll and pitch angle and stores it as
            - the zero angle which is used to correct all measurements
            - for user interfacing and for blimp stabilization
        """
        
        # protected read
        [r, p, _] = self.get_euler()

        # protected write
        self.lock_rot0.acquire()
        self.rot0[:2] = [r, p]
        self.lock_rot0.release()
        
    def zero_z_rot(self):
        """ grabs the current yaw angle and stores it as the zero angle 
            - which is used to correct all measurements for user 
            - interfacing and for blimp stabilization
        """
        
        # protected read
        [_, _, yw] = self.get_euler()

        # protected write
        self.lock_rot0.acquire()
        self.rot0[2] = yw
        self.lock_rot0.release()

import multiprocessing as mp
import numpy as np
import serial

from multiprocessing import shared_memory as sm
from numpy import pi
from scipy.spatial.transform import Rotation as R


def wrap(ang):
    ang += 2*pi*(ang < -pi) - 2*pi*(ang > pi)
    return ang
    

def euler(quat):
    return R.from_quat(quat).as_euler('xyz')


def blimp_coordinates(eul, rot0):
    # transform to blimp coordinate system
    eul[0] += pi
    eul[0] -= 2*pi*(eul[0] > pi)
    eul[0:1] *= -1
    
    # transform with zero angles (no lock needed)
    eul -= rot0

    return wrap(eul)


def create_serial(port):
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 921600
    ser.write_timeout = 0
    ser.open()
    
    lock = mp.Lock()
    
    return (ser, lock)

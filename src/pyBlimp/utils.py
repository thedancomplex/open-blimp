import multiprocessing as mp
import numpy as np
import serial
import yaml

from multiprocessing import shared_memory as sm
from numpy import pi
from scipy.spatial.transform import Rotation as R


def wrap(ang):
    return ang + 2*pi*(ang < -pi) - 2*pi*(ang > pi)


def quat2euler(quat):
    return R.from_quat(quat).as_euler('xyz')


def create_serial(port):
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 921600
    ser.write_timeout = 0
    ser.open()
    return (ser, mp.Lock())
    
def read_config(path):
    return yaml.safe_load(open(path))
        

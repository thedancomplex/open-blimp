import inspect
import multiprocessing as mp
import numpy as np
import os
import socket
import signal
import struct

from multiprocessing import shared_memory as sm

class ViconInterface:
    def __init__(self):
        cur_path = inspect.getfile(self.__class__)
        dir_path = cur_path.split('viconReader.py')[0]
        total_path = os.path.join(dir_path, 'names.txt')

        my_file = open(total_path, "r")
        data =  my_file.read()
        names = data.split('\n')

        self.smems = {}
        self.poses = {}
        self.locks = {}
        
        sm_names = []
        sm_locks = []

        self.running = sm.SharedMemory(create=True, size=1)
        self.running.buf[0] = True

        for name in names:
            if len(name) == 0: continue
            print("Latching to", name)
            self.smems[name] = sm.SharedMemory(create=True, size=64)
            self.poses[name] = np.ndarray(8, dtype=np.float64, buffer=self.smems[name].buf)
            self.locks[name] = mp.Lock()
            
            sm_names.append(self.smems[name].name)
            sm_locks.append(self.locks[name])

        args = (names, sm_names, sm_locks, self.running.name)
        self.pVicon = mp.Process(target=handle_ViconReader, args=args)
        self.pVicon.start()

        # start the second process
        cur_path = inspect.getfile(self.__class__)
        dir_path = cur_path.split('viconReader.py')[0]
        total_path = os.path.join(dir_path, 'rosReader.py')
        os.system("/usr/bin/python3 "+str(total_path)+" "+" &")

    def shutdown(self):
        self.running.buf[0] = False
        self.pVicon.join()
        
        self.running.close()
        self.running.unlink()
        for name in self.smems:
            self.smems[name].close()
            self.smems[name].unlink()

    def get_pose(self, name):
        if type(self.running.buf) is None: return None

        self.locks[name].acquire()
        pose = self.poses[name]
        self.locks[name].release()
        return pose

def handle_ViconReader(names, sm_names, sm_locks, run_name):
    V = _ViconReader(names, sm_names, sm_locks, run_name)
    V.run()

class _ViconReader:
    def __init__(self, names, sm_names, sm_locks, run_name):
        self.running = sm.SharedMemory(run_name)
        self.smems = {}
        self.poses = {}
        self.locks = {}

        signal.signal(signal.SIGINT, self.handler)

        for name, sm_name, lock in zip(names, sm_names, sm_locks):
            self.smems[name] = sm.SharedMemory(sm_name)
            self.poses[name] = np.ndarray(8, dtype=np.float64, buffer=self.smems[name].buf)
            self.locks[name] = lock

    def handler(self, signum, frame):
        self.running.buf[0] = False
    
    def run(self):
        sock_rcv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock_rcv.settimeout(0.5)        
        sock_rcv.bind(('127.0.0.1', 4443))

        # wait until shutdown flag is called
        while self.running.buf[0]:
            try: data, _ = sock_rcv.recvfrom(1000)
            except: data = None

            if data is not None:
                # split into pose and topic data
                pose_data  = data[:64]
                topic_data = data[64:]
                
                # interpret and store in shared memory
                topic = "".join(map(chr, topic_data))
                self.locks[topic].acquire()
                self.poses[topic][:] = struct.unpack("<8d", pose_data)
                self.locks[topic].release()

        # send shutdown request
        sock_done = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock_done.sendto('DONE'.encode("utf-8"), ("127.0.0.1", 4444))

        # cleanup
        sock_done.close()
        sock_rcv.close()
        self.running.close()
        for name in self.smems:
            self.smems[name].close()


#!/usr/bin/python
import inspect
import os
import rospy
import rosgraph
import signal
import socket
import struct

from geometry_msgs.msg import TransformStamped


class RosReader:
    def __init__(self):
        # class to handle transferring vicon data to python3
        # - uses a local socket interface to transfer the data

        signal.signal(signal.SIGINT, self.handler)

        cur_path = inspect.getfile(self.__class__)
        dir_path = cur_path.split('rosReader.py')[0]
        total_path = os.path.join(dir_path, 'names.txt')

        my_file = open(total_path, "r")
        data =  my_file.read()
        names = data.split('\n')
        
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.running = True

        sock_rcv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock_rcv.settimeout(0.5)        
        sock_rcv.bind(('127.0.0.1', 4444))

        ros_init = False
        while self.running:
            try: data, _ = sock_rcv.recvfrom(4)
            except: data = None

            if data is not None:
                msg = "".join(map(chr, data))
                if msg == 'DONE': self.running = False


            if not ros_init:
                socket.setdefaulttimeout(0.1)
                ros_online = rosgraph.is_master_online()
                socket.setdefaulttimeout(None)            
                if ros_online:
                    ros_init = True
                    rospy.init_node('viconListener', anonymous=True)
                    for name in names:
                        if len(name) > 0:
                            print("Subscribing to", name)
                            rospy.Subscriber(name, TransformStamped, self.callback, callback_args=name)

        self.sock_send.close()
        sock_rcv.close()        

    def handler(self, signum, frame):
        self.running = False

    def callback(self, data, topic):
        if not self.running: return

        # parse data into 2 tuples
        t = data.header.stamp.to_sec()
        x = data.transform.translation.x
        y = data.transform.translation.y
        z = data.transform.translation.z
        qx = data.transform.rotation.x
        qy = data.transform.rotation.y
        qz = data.transform.rotation.z
        qw = data.transform.rotation.w
                
        pose = (t, x, y, z, qx, qy, qz, qw)

        # send data over socket
        pose_bytes = struct.pack("<8d", *pose)
        pose_bytes += topic.encode("utf-8")
        self.sock_send.sendto(pose_bytes, ("127.0.0.1", 4443))

if __name__ == "__main__":
    R = RosReader()

import rclpy

from std_msgs.msg import Int8MultiArray

g_node = None
sub = None


MOT_0 = 0
MOT_1 = 1
MOT_2 = 2
MOT_3 = 3
MOT_4 = 4
MOT_5 = 5
MOT_DIR = 6

i = 0
rxdata = None
def feedforward_callback(data):
    global i
    global rxdata
    i+=1
    print(i)
    print(data)
    rxdata = data.data 

def init():
    global g_node
    global sub
    print('Ros2 to Blimp setup Start')
    rclpy.init()
    g_node = rclpy.create_node('open_blmp')
    sub = g_node.create_subscription(Int8MultiArray, '/open_blimp/ref',feedforward_callback,10)
    print('Ros2 to Blimp setup Complete')

def get():
    global g_node
    global i
    i+=1
    print(i)
    rclpy.spin_once(g_node,timeout_sec=0)
    return rxdata
    #rclpy.spin_once(g_node, timeout_sec=0.0001)

def doPrint():
    print('dan')
    return 42

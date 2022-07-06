from pyBlimp.blimp import *

# setup serial device
port = "xxx"
ser = create_serial(port)

# build the blimp object
pi_ports = [1111, 1112, 1113]
b = Blimp(1, port, ser, pi_ports)

# setup exit on ctrl-c
running = True
def exit_handle(signum, frame):
    global running
    running = False
    
signal.signal(signal.SIGINT, exit_handle)

# main loop
while running:
    # main application code
    b.poll()
    
    
b.pi.shutdown()

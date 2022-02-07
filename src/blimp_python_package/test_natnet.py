from utils.NatNetBlimp import Natnet_blimp
import time

class test:
    def __init__(self):
        self.NN = Natnet_blimp("192.168.1.217", "192.168.1.214")
        self.position = list()
    
    def receive_rigid_body_frame(self, new_id, position, rotation ):
        self.position.append(position)
        print( "Received frame for rigid body", new_id," ",position," ",rotation )

    def runNatNet(self):
        self.NN.streaming_client.rigid_body_listener = self.receive_rigid_body_frame
        self.NN.streaming_client.run()
    
    def stopNatNet(self):
        self.NN.streaming_client.shutdown()
        print(self.position)

if __name__ == "__main__":
    myTest = test()
    myTest.runNatNet()
    time.sleep(5)
    myTest.stopNatNet()

from SimPy.Simulation import *

class Device(Process):

    def __init__(self, ID):
        Process.__init__(self, name="Device"+str(ID))
        self.ID = ID

    def run(self):
        # Implementation left to subclasses
        pass
    
    def addLink(self, link):
        # Implementation left to subclasses
        pass
     
    def receivePacket(self, packet):
        # Implementation left to subclasses
        pass
    
    def sendPacket(self, packet, link):
        # Implementation left to subclasses
        pass
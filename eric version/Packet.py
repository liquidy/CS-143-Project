from SimPy.Simulation import *
from Devices import *
from Global import *

####################################################################################    
# PACKET
####################################################################################

#packet:
#- packet is a process
#- source creates packet object (at first packet is not activated), enqueues it in the #link
#- link then tells it where it is going (in terms of next device) and tells it how long #to wait, and then activates the packet
#- packet waits for the needed time
#- once it has waited for that time it enqueues itself in that appropriate device #(router) and goes to sleep
#- packet needs public router object
#- when packet wakes up, it is because a link has reactivated it

#Packet:
#- int ID, int size, int timeSent
#- int sourceID, int destinationID
#- Boolean isRouterMessage, Boolean acknowledgement
#- Object Message (RouterMessage, Other)

class Packet(Process):
    # To create a Packet(process), use p1 = Packet(1, 20, 1)
    # to create a packet with ID = 1, size = 20, timeSent = 1
    def __init__(self, packetID, timeSent, sourceID, desID, isRouterMesg, isAck, myMessage, globs):
        Process.__init__(self, name="Packet" + str(packetID))
        self.packetID= packetID
        self.globs = globs
        self.size = globs.PACKET_SIZE
        self.timeSent = timeSent
        self.sourceID = sourceID
        self.desID = desID
        self.isRouterMesg = isRouterMesg
        self.isAck = isAck
        self.myMessage = myMessage
        self.propTime = None
        self.device = None
        
    def run(self):
        while True:
            yield passivate, self
            # When link starts transmitting, it activates packet.
            # Packet holds for the propTime
            yield hold, self, self.propTime
            # It then euqueues into device's queue
            if not self.device.active:
                reactivate(self.device)
                self.device.active = True
            self.device.receivePacket(self)

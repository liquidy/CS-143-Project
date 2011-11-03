from SimPy.Simulation import *

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
    def __init__(self, packetID, timeSent, sourceID, desID, isRouterMesg, isAck, myMessage):
        Process.__init__(self, name="Packet"+str(packetID))
        self.packetID= packetID
        self.size = 8000
        self.timeSent = timeSent
        self.sourceID = sourceID
        self.desID = desID
        self.isRouterMesg = isRouterMesg
        self.isAck = isAck
        self.myMessage = myMessage
        self.propTime = None
        self.device = None
        
    # to activate the process, call activate(p1, p1.run())
    # I understand why link should be active-passivate-active...
    # but I think packet could always stay active, I don't think I see
    # why we need to make it passivate and wake up again.
    def run(self):
        while True:
            yield passivate, self
            yield hold, self, self.propTime
            if not self.device.busy:
                reactivate(device)
            self.device.receivePacket(self)
        
    
        
        
        
        
        
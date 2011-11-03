from SimPy.Simulation import *

class Source(Device):

    def __init__(self, ID, destinationID, flowRate, bitsToSend, congestionAlgID,m):
        super(self, ID)
        self.destinationID = destinationID
        self.bitsToSend = bitsToSend
        self.flowRate = flowRate
        self.congestionAlgID = congestionAlgID
        self.roundTripTime = 0
        self.windowsSize = 100
        self.currentPacketID = 0
        self.outstandingPackets = {}
        self.link = None
        self.active = False
        self.sendRateMonitor = m
        self.numPacketsSent = 0

    def addLink(self, link):
        self.link = link

    def run(self):
        while True:
            if len(outstandingPackets) < windowSize:
                yield passivate, self
            if not self.link.active:
                reactivate(link)
            packet = createPacket()
            self.sendPacket(packet)
            numPacketsSent = numPacketsSent + 1
            sendRateMonitor.observe(numPacketsSent/now())
            outstandingPackets[packet] = True;
            
    def createPacket(self):
        message = "Packet " + str(self.currentPacketID) + "'s data goes here!"
        newPacket = Packet(self.currentPacketID, now(), self.ID, 
                           self.destinationID, False, False, message)
        self.currentPacketID += 1
        return newPacket
    
    def sendPacket(self, packet):
        link.queue.append(packet)
        
    def receivePacket(self, packet):
        if not packet.isAck:
            return
        del outstandingPackets[packet]
        
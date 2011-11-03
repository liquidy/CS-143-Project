from SimPy.Simulation import *
import Device

class Destination(Process, Device):

    def __init__(self, ID, sourceID):
        super(self, ID) # call Device
        self.sourceID = sourceID
        self.numPacketsReceived = 0
        self.totalPacketDelay = 0   # calculate the sum of the total packet delays
        self.link = None
        self.active = False
        self.receivedPackets = []  # stores all the rec'd packets
        self.currentAckPacketID = 0


    def addLink(self, link):
        self.link = link
        
        
    # process packets as they arrive
    def run(self):
        while True:
            # passivate until receivePacket is called
            yield passivate
            
            
    # called by link when it is time for packet to get to dest
    def receivePacket(self, packet):
        receivedTime = now()
        self.receivedPackets.append((receivedTime, packet)) # list holds (time, packet) tuples
        numPacketsReceived += 1
        
        # calculate the delay
        currDelay = receivedTime - packet.timeSent
        totalPacketDelay += currDelay

        # send ack
        acknowledge(packet)
        
        print 'Received packet: ' + packet.ID + ' at time ' + receivedTime
        
        # collect other stats: Ben?
    
    
    # sends ack once it receives packet
    def acknowledge(packet):
        ack = createAckPacket()

        # send the ack packet (append it to this dest's link's queue)
        self.link.queue.append(ack)
        
    
    def createAckPacket(self):
        #(packetID, timeSent, sourceID, desID, isRouterMesg, isAck, myMessage, monitor)
        message = "Acknowledgement packet " + str(self.currentAckPacketID) + "'s data goes here!"

        # need to instantiate monitor object to pass to ack packet
        newPacket = Packet(self.currentAckPacketID, now(), self.ID, 
                           packet.sourceID, False, True, message, monitor)

        self.currentAckPacketID += 1
        return newPacket  
from SimPy.Simulation import *
from Devices import *
from Packet import *
from Global import *

################################################################################
#   RoutingTimer                                                               #
################################################################################
#
#   Timer process for a Router which periodically triggers an event handler
#   for probing link delays while not rerouting and requesting link-state
#   data while rerouting.
class RoutingTimer(Process):

    def __init__(self, router):
        Process.__init__(self, name = "RoutingTimer " + str(router.ID))
        self.router = router
        self.time = router.globs.PROBE_RATE
        
    def run(self):
        while True:
            yield hold, self, self.time
            if self.router.globs.flowsDone == self.router.globs.numFlows: # If the simulation is over, stop
                yield passivate, self
            self.router.timerHandler()

            
################################################################################
#   SourceTimer                                                                #
################################################################################

# There is a SourceTimer object associated with each packet, so
# this is instantiated every time a packet is sent. This timer waits the timeout
# time set by the source, then resends packet while it is in list of outstanding
# packets. Once it is no longer in that list, the timer just passivates.
class SourceTimer(Process):

    def __init__(self, packet, time, source):
        Process.__init__(self, name = "Timer" + str(packet.packetID))
        self.packet = packet     # the packet which this timer is associated with
        self.time = time         # the length of time before timer times out
        self.source = source     # the object which sends the packets
    
    #
    # Called when Timer is activated.
    #
    def run(self):
        # While we still have outstanding packets
        while self.packet.packetID in self.source.outstandingPackets:
            # wait until we reacht the timeout time
            yield hold, self, self.time
            # at this point we have a timeout
            
            # While we havent received an ack for the packet
            if self.packet.packetID in self.source.outstandingPackets:
                # When time out, update variables to slow start
                self.source.enabledCCA = False
                self.source.fastRecovery = False
                self.source.ssthresh /= 2
                self.source.windowSize = 1
                
                self.source.missingAck += 1
                    
                # Add packet to source's queue to retransmit the timeout packet
                self.source.toRetransmit.append(self.createPacket(self.packet))
                
                # Make sure source is activated
                if not self.source.active:
                    self.source.active = True
                    reactivate(self.source)
        yield passivate, self
            
    # 
    # Create a copy of packet with this id
    #
    def createPacket(self, packet):
        message = "Packet " + str(packet.packetID) + "'s data goes here!"
        newPacket = Packet(packet.packetID, now(), packet.sourceID,
                           packet.desID, packet.isRouterMesg, packet.isAck, message, self.source.globs)
        activate(newPacket, newPacket.run())
        return newPacket

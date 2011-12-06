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
        Process.__init__(self, name="RoutingTimer " + str(router.ID))
        self.router = router
        self.time = router.globs.PROBE_RATE
        
    def run(self):
        while True:
            yield hold, self, self.time
            if router.globs.flowsDone == router.globs.numFlows: # If the simulation is over, stop
                yield passivate, self
            self.router.timerHandler()

            
            
# SourceTimer
# this is called every time a packet is sent. it waits the timeout
# set by the source, then resends packet while it is in list of outst
# packets. once it is no longer in that list, just passivate
class SourceTimer(Process):
    # packet is the packet for which this timer is associated with
    # time is the length of time before timer times out
    # source is the object which sends the packets
    def __init__(self, packet, time, source):
        Process.__init__(self, name="Timer" + str(packet.packetID))
        self.packet = packet
        self.time = time
        self.source = source
    
    def run(self):
        while self.packet.packetID in self.source.outstandingPackets:
            yield hold, self, self.time
            
            # while we havent received an ack for the packet
            if self.packet.packetID in self.source.outstandingPackets:
                # When time out, update variables to slow start
                self.source.enabledCCA = False
                self.source.fastRecovery = False
                self.source.ssthresh /= 2
                self.source.windowSize = 1
                
                self.source.missingAck += 1
                    
                #Add packet to source's queue for retransmit the timeout packet
                self.source.toRetransmit.append(self.createPacket(self.packet))
                if not self.source.active:
                    self.source.active = True
                    reactivate(self.source)
        yield passivate, self
            
    # copy of packet with this id
    def createPacket(self, packet):
        message = "Packet " + str(packet.packetID) + "'s data goes here!"
        # packetId, timesent, sourceID, destid, isroutermsg, isack, msg
        newPacket = Packet(packet.packetID, now(), packet.sourceID,
                           packet.desID, packet.isRouterMesg, packet.isAck, message, self.source.globs)
        activate(newPacket, newPacket.run())
        return newPacket

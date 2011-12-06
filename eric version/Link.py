from SimPy.Simulation import *
from Devices import *
from Packet import *
from Global import *

####################################################################################    
# LINK
####################################################################################

#
# Link class - extends Process. Links connect Sources, Routers, and Destinations, and
# transmit Packets.
#
class Link(Process):

    def __init__(self, linkRate, start, end, propTime, bufferCapacity, buffMonitor, dropMonitor, flowMonitor):
        Process.__init__(self)
        self.queue = []
        self.linkRate = linkRate
        self.start = start
        self.end = end
        self.propTime = propTime
        self.bufferCapacity = bufferCapacity
        self.buffMonitor = buffMonitor
        self.dropMonitor = dropMonitor
        self.flowMonitor = flowMonitor
        self.droppedPackets = 0
        self.packetsSent = 0        
        self.active = False

    # if link is active, self.active = True, otherwise, self.active = False
    # Source can use this attribute to decide if it needs to reactivate the  
    # link
    def run(self):
        self.active = True
        while True:
            # link goes to sleep if there's no more stuff to transmit
            if self.queue == []:
                self.active = False
                yield passivate, self
                self.active = True
            assert len(self.queue) > 0

            packet = self.queue.pop(0)
            # wait for trans time
            yield hold, self, packet.size / float(self.linkRate)
            # update stats
            self.packetsSent += 1
            if not now() == 0:
                self.flowMonitor.observe(packet.size * self.packetsSent / float(now()))
            # packet start propagate in the link
            packet.propTime = self.propTime
            packet.device = self.end
            reactivate(packet)

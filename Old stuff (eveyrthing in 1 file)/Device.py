from SimPy.Simulation import *
from collections import deque
import numpy as py
import matplotlib.pyplot as plt
import math
import sys

PACKET_SIZE = 8000
#INIT_WINDOW_SIZE = 1000
INIT_WINDOW_SIZE = 1
THRESHOLD = 100
ACK_TIMEOUT = 1000 # NEED TO ESTIMATE LATER
DYNAMIC_ROUTING = False
PROBE_DROP_DELAY = 30
PROBE_SAMPLE_SIZE = 30
PROBE_RATE = 15
DEFAULT_ALPHA = 50
NUM_PACKETS_TO_TRACK_FOR_RTT = 10
CONGESTION_CONTROL_ALGORITHM = "VEGAS"

####################################################################################
# DEVICE
####################################################################################

#
# Describes a superclass from which Router, Source, and Destination extend.
#
class Device(Process):

    # Each Device has an ID.
    def __init__(self, ID):
        Process.__init__(self, name="Device" + str(ID))
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


####################################################################################    
# DESTINATION
####################################################################################

# 
# Destination class - extends Device.
#
class Destination(Device):
    
    # Each Destination keeps track of its ID, the ID of its corresponding 
    # Source, the 'throughput' monitor, and the 'packet delay' monitor.
    def __init__(self, ID, sourceID, throughput, packetDelay):
        Device.__init__(self, ID)        # call Device's constructor
        self.sourceID = sourceID
        self.numPacketsReceived = 0
        self.totalPacketDelay = 0        # calculate the sum of the total packet delays
        self.link = None                 # keeps track of the link it is connected to
        self.active = False              # is this process active?
        self.receivedPackets = []        # list of packetID's   
        self.throughput = throughput     # monitor - collects stats
        self.packetDelay = packetDelay   # monitor - collects stats
        self.packet = None               # the current packet we are dealing with
        self.missingPackets = []         # which packets have we not received yet?
        self.currentPacketIDToAck = -1     # keeps track of the next packet ID to acknowledge
        self.highestReceivedPacket = -1

    # Attach a Link to this Destination.
    def addLink(self, link):
        self.link = link
        
    # Process packets as they arrive
    def run(self):
        self.active = True
        while True:
            # passivate until receivePacket is called
            self.active = False
            yield passivate, self
            self.active = True
            
            # collect stats
            receivedTime = now()
            self.link.buffMonitor.observe(len(self.link.queue))
            self.link.dropMonitor.observe(self.link.droppedPackets)
            
            # at this point we have a packet to deal with
            assert(self.packet is not None)
            
            # dont do anything if this is a router packet
            if not self.packet.isRouterMesg:
                # add to the aggregate delay across all packets
                self.totalPacketDelay += (now() - self.packet.timeSent)
                self.numPacketsReceived += 1
                
                # Receive the packet
                self.receivedPackets.append(self.packet.packetID)
                if self.packet.packetID in self.missingPackets:
                    self.missingPackets.remove(self.packet.packetID)
                
                # Non consecutive packets must be added to missing
                for i in range(self.highestReceivedPacket+1,self.packet.packetID):
                    self.missingPackets.append(i)
                
                # Update the highest packets received    
                if self.highestReceivedPacket < self.packet.packetID:
                    self.highestReceivedPacket = self.packet.packetID
                    
                # Ack the smallest missing packet
                if len(self.missingPackets) == 0:
                    self.currentPacketIDToAck = self.highestReceivedPacket
                else:
                    self.currentPacketIDToAck = min(self.missingPackets)-1
                    
                # send ack to currentAckPacketID packet and collect stats (happens always)
                self.acknowledge()
                                    
                if not now() == 0:
                    # collect stats
                    self.packetDelay.observe(self.totalPacketDelay / float(self.numPacketsReceived))
                    self.throughput.observe(self.numPacketsReceived * self.packet.size / float(now()))
                    
            self.packet = None
     
    # Called by link when it is time for a packet to reach this Destination.
    def receivePacket(self, packet):
        self.packet = packet
    
    # sends ack once it receives packet
    def acknowledge(self):
        if not self.link.active:
            reactivate(self.link)
            self.link.active = True
        # send the ack packet (append it to the queue of this object's link)
        self.link.queue.append(self.createAckPacket())
        
    # Generate a new ack packet.
    def createAckPacket(self):
        message = "Acknowledgement packet " + str(self.currentPacketIDToAck) + "'s data goes here!"
        # create the packet object with the needed ack packet id, source id, etc.
        newPacket = Packet(self.currentPacketIDToAck, now(), self.ID, 
                           self.sourceID, False, True, message)
        activate(newPacket, newPacket.run())
        return newPacket    

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
            if self.queue == []:
                self.active = False
                yield passivate, self
                self.active = True
            assert len(self.queue) > 0

            packet = self.queue.pop(0)
            # wait for trans time
            yield hold, self, packet.size / float(self.linkRate)
            self.packetsSent += 1
            if not now() == 0:
                self.flowMonitor.observe(packet.size * self.packetsSent / float(now()))
            # packet start propagate in the link
            packet.propTime = self.propTime
            packet.device = self.end
            reactivate(packet)


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
    def __init__(self, packetID, timeSent, sourceID, desID, isRouterMesg, isAck, myMessage):
        Process.__init__(self, name="Packet" + str(packetID))
        self.packetID= packetID
        self.size = PACKET_SIZE
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
            if not self.device.active:
                reactivate(self.device)
                self.device.active = True
            self.device.receivePacket(self)

class RoutingTimer(Process):

    def __init__(self, router):
        Process.__init__(self, name="RoutingTimer " + str(router.ID))
        self.router = router
        self.time = PROBE_RATE
        
    def run(self):
        while True:
            yield hold, self, self.time
            if flowsDone == numFlows:
                yield passivate, self
            self.router.timerHandler()
            
class Router(Device):
    
    # network - adjancency matrix of (monitored, link rate, propagation delay)
    def __init__(self, id, network):
        Device.__init__(self, id)
        self.network = network
        self.networkSize = len(self.network)
        self.buffer = []
        self.active = False
        self.rerouting = False
        self.links = [] # to be filled by global initializer
        self.linkMap = [None] * self.networkSize # linkMap[i] is a link to node i
        self.delays = [] # delays[i] is a list of measured packet delays to node i
        for i in range(self.networkSize):
            self.delays.append([])
        self.delayData = [] # data sent to other routers during rerouting
        self.haveData = [] # haveData[i] indicates whether the link state data for node i is known
        self.haveAck = [] # haveAck[i] indicates whether a topAck has been recieved from node i
        self.routingTable = [None] * self.networkSize
        self.timer = RoutingTimer(self)
    
    def run(self):
        self.active = True
        if DYNAMIC_ROUTING:
            activate(self.timer, self.timer.run())
        self.mapLinks()
        self.dijkstra()
        while True:
            if len(self.buffer) == 0:
                for link in self.links:
                    link.buffMonitor.observe(len(link.queue))
                    link.dropMonitor.observe(link.droppedPackets)
                self.active = False
                yield passivate, self
                self.active = True
            pkt = self.buffer.pop(0)
            dest = pkt.desID
            if dest == self.ID:
                self.processMessage(pkt)
            else:
                self.sendPacket(pkt, self.routingTable[dest])
            
    def sendPacket(self, pkt, link):
        if len(link.queue) < link.bufferCapacity:
            if not link.active:
                reactivate(link)
                link.active = True
            link.queue.append(pkt)
        else:
            link.droppedPackets += 1
            
    def receivePacket(self, pkt):
            self.buffer.append(pkt)
    
    def addLink(self, link):
        self.links.append(link)
        
    def mapLinks(self):
        for link in self.links:
            self.linkMap[link.end.ID] = link
    
    def dijkstra(self):
        # initialize a list of (predecessor, distance, deviceID) triples
        paths = []
        for i in range(self.networkSize):
            paths.append((None, None, i))
        paths[self.ID] = (self.ID, 0, self.ID)
        # initialize a list of IDs of undiscovered nodes
        left = []
        for i in range(self.networkSize):
            left.append(i)
        
        while len(left) > 0:
            # find closest undiscovered node
            minDist = None
            nextNode = None
            for node in left:
                dist = paths[node][1]
                if not dist == None:
                    if minDist == None or minDist > dist:
                        minDist = dist
                        nextNode = node
                        
            # discover closest node and update shortest distances to its neighbors
            left.remove(nextNode)
            for i in range(self.networkSize):
                link = self.network[nextNode][i]
                if len(link) > 1: # if a link exists from nextNode to node i
                    dist = minDist + link[2]
                    if dist < paths[i][1] or paths[i][1] == None: # if we found a shorter path to node i
                        paths[i] = (nextNode, dist, i)
        
        for i in range(len(paths)):
            node = paths[i]
            # find next step toward node i from self
            while node[0] is not self.ID:
                node = paths[node[0]]
            # update routing table
            self.routingTable[i] = self.linkMap[node[2]]
            
        self.rerouting = False
        

    def processMessage(self, pkt):
        source = pkt.sourceID
        (type, data) = pkt.myMessage
        if type == "probe" and not self.rerouting:
            # no data; send probeAck to source with packet delay
            msg = ("probeAck", float(now()) - pkt.timeSent)
            ack = Packet("Probe %d %d" % (self.ID, source), now(), self.ID, source, True, True, msg)
            activate(ack, ack.run())
            self.sendPacket(ack, self.routingTable[source])
        elif type == "probeAck" and not self.rerouting:
            # data is packet delay over link to source
            self.delays[source].append(data)
            if len(self.delays[source]) >= PROBE_SAMPLE_SIZE:
                self.initiateReroute()
        elif type == "reroute":
            # data is list of (start, end, delay) tuples;
            # if not rerouting, begin rerouting and broadcast delays;
            # send topAck to source;
            # process the recieved data and check if done rerouting
            if self.rerouting == False:
                self.initiateReroute()
            pkt = Packet("Delay Data at %d" % (self.ID), now(), self.ID, source, True, True, ("topAck", self.delayData))
            activate(pkt, pkt.run())
            self.sendPacket(pkt, self.routingTable[source])
            if not self.haveData[source]:
                self.haveData[source] = True
                self.updateNetwork(data)
            if self.haveData == [True] * self.networkSize and self.haveAck == [True] * self.networkSize:
                self.dijkstra()
        elif type == "topology":
            # data is list of (start, end, delay) tuples;
            # send topAck to source and and check if done rerouting
            pkt = Packet("Delay Data at %d" % (self.ID), now(), self.ID, source, True, True, ("topAck", self.delayData))
            activate(pkt, pkt.run())
            self.sendPacket(pkt, self.routingTable[source])
            if self.rerouting:
                if not self.haveData[source]:
                    self.haveData[source] = True
                    self.updateNetwork(data)
                if self.haveData == [True] * self.networkSize and self.haveAck == [True] * self.networkSize:
                    self.dijkstra()
        elif type == "topAck" and self.rerouting:
            # data is list of (start, end, delay) tuples;
            # check if data is new and process if so;
            # source has recieved delay data, check if done rerouting
            self.haveAck[source] = True
            if not self.haveData[source]:
                self.haveData[source] = True
                self.updateNetwork(data)
            if self.haveData == [True] * self.networkSize and self.haveAck == [True] * self.networkSize:
                self.dijkstra()
    
    def initiateReroute(self):
        self.rerouting = True
        self.haveData = [False] * self.networkSize
        self.haveData[self.ID] = True
        self.haveAck = [False] * self.networkSize
        self.haveAck[self.ID] = True
        self.delayData = []
        for i in range(self.networkSize):
            delays = self.delays[i]
            if delays != []:
                self.delayData.append((self.ID, i, sum(delays)/float(len(delays))))
            self.delays[i] = []
        self.updateNetwork(self.delayData)
        for i in range(self.networkSize):
            if i is not self.ID:
                if not (nodes[i][1] or nodes[i][2]):
                    pkt = Packet("Delay Data at %d" % (self.ID), now(), self.ID, i, True, False, ("reroute", self.delayData))
                    activate(pkt, pkt.run())
                    self.sendPacket(pkt, self.routingTable[i])
                else:
                    self.haveData[i] = True
                    self.haveAck[i] = True
                
    def updateNetwork(self, data):
        for datum in data:
            (start, end, delay) = datum
            self.network[start][end][2] = delay
                
    def timerHandler(self):
        if not self.rerouting:
            # send probes
            for link in self.links:
                dst = link.end.ID
                if len(link.queue) >= link.bufferCapacity:
                    self.delays[dst].append(PROBE_DROP_DELAY)
                pkt = Packet("Probe %d %d" % (self.ID, dst), now(), self.ID, dst, True, False, ("probe", None))
                activate(pkt, pkt.run())
                self.sendPacket(pkt, link)
        else:
            # resent unacknowledged data
            for i in range(self.networkSize):
                if not self.haveAck[i]:
                    pkt = Packet("Delay Data at %d" % (self.ID), now(), self.ID, i, True, False, ("topology", self.delayData))
                    activate(pkt, pkt.run())
                    self.sendPacket(pkt, self.routingTable[i])
    

# Source
class Source(Device):

    def __init__(self, ID, destinationID, bitsToSend, congControlAlg, startTime, sendRateMonitor, windowSizeMonitor):
        Device.__init__(self, ID)
        self.destinationID = destinationID
        self.bitsToSend = bitsToSend
        self.congControlAlg = CONGESTION_CONTROL_ALGORITHM
        self.roundTripTime = 0
        self.windowSize = INIT_WINDOW_SIZE
        self.currentPacketID = 0
        self.outstandingPackets = {}
        self.toRetransmit = []
        self.link = None
        self.active = False
        self.sendRateMonitor = sendRateMonitor
        self.windowSizeMonitor = windowSizeMonitor
        self.startTime = startTime
        self.numMissingAcks = 5
        self.missingAck = 0
        self.numPacketsSent = 0
        self.timeout = ACK_TIMEOUT
        self.enabledCCA = False
        self.alpha = DEFAULT_ALPHA
        self.ssthresh = THRESHOLD
        
        # Variables for congestion control
        self.acks = {}
        self.mostRecentAck = -1
        self.fastRecovery = False
        self.rttMin = 0 # RTT for first packet
        self.rtt = 0 # Average RTT for the last "numPacketsToTrack" packets
        self.packetRttsToTrack = deque() # Queue tracking the last "numPacketsToTrack" packet RTTs
        self.numPacketsToTrack = NUM_PACKETS_TO_TRACK_FOR_RTT  # Number of packets to track in packetRttsToTrack
        self.timePacketWasSent = {} # Dict that records the time that a packet was set

    def addLink(self, link):
        self.link = link
    
    def run(self):
        self.active = True
        packetIdToRetransmit = -1
        yield hold, self, self.startTime
        while True:       
            packetIdToRetransmit = self.getPacketIdToRetransmit()
            
            # First time we get 3DA
            if self.enabledCCA and not self.fastRecovery and packetIdToRetransmit!= -1:
                # Enter Fast Recovery if necessary
                self.fastRecovery = True
                self.ssthresh = math.max(math.ceil(self.windowSize/2), 2)
                self.windowSize = self.ssthresh + 3               
            
            # Further times we get 3DA
            if self.fastRecovery and packetIdToRetransmit != -1:
                # For every 3 dup acks received, get the packetID for retransmit
                self.acks[packetIdToRetransmit-1] = 0
                assert(packetIdToRetransmit != -1)
                
                # Create a new packet based on the packetID, and resent the packet
                message = "Packet " + str(packetIdToRetransmit) + "'s data goes here!"
                # packetId, timesent, sourceID, destid, isroutermsg, isack, msg
                newPacket = Packet(packetIdToRetransmit, now(), self.ID,
                        self.destinationID, False, False, message)
                activate(newPacket, newPacket.run())
                self.sendPacketAgain(newPacket)
                yield hold, self, newPacket.size/float(self.link.linkRate)
            # resend anything, if need to
            elif len(self.toRetransmit) > 0:
                (didtransmit, p) = self.retransmitPacket()
                if (didtransmit): # if transmitted, wait
                    yield hold, self, p.size/float(self.link.linkRate)
                    # collect stats
                    if not now() == 0:
                        self.link.buffMonitor.observe(len(self.link.queue))
                        self.link.dropMonitor.observe(self.link.droppedPackets)
                        self.sendRateMonitor.observe(PACKET_SIZE*self.numPacketsSent / float(now()))
                
            # If everything has been sent, go to sleep
            elif (PACKET_SIZE * self.numPacketsSent >= self.bitsToSend):
                self.active = False
                yield passivate, self
                self.active = True
            # otherwise, send new packets!
            elif len(self.outstandingPackets) < self.windowSize:
                # send a new packet
                packet = self.createPacket()
                self.sendPacket(packet)
                # wait
                yield hold, self, packet.size/float(self.link.linkRate)
                # collect stats
                if not now() == 0:
                    self.link.buffMonitor.observe(len(self.link.queue))
                    self.link.dropMonitor.observe(self.link.droppedPackets)
                    self.sendRateMonitor.observe(PACKET_SIZE*self.numPacketsSent / float(now()))
            # nothing to retransmit and cannot send new packets
            else:
                self.active = False
                yield passivate, self
                self.active = True
            
    def createPacket(self):
        message = "Packet " + str(self.currentPacketID) + "'s data goes here!"
        newPacket = Packet(self.currentPacketID, now(), self.ID,
                           self.destinationID, False, False, message)
        self.currentPacketID += 1
        activate(newPacket, newPacket.run())
        return newPacket
    
    # figures out if we need to retransmit anything, and retransmits it if so
    def retransmitPacket(self):
        packet = None # the packet to send
        # check if we need to resend anything, do nothing if nothing to resend
        if len(self.toRetransmit) > 0:
            # keep popping packets off toRetr until we find one that is outstanding
            while len(self.toRetransmit) > 0:
                pack = self.toRetransmit.pop()
                if (pack.packetID in self.outstandingPackets):
                    packet = pack
                    break
            if (packet != None):
                # have a single packet which we need to resend
                packet.timeSent = now()
                self.sendPacketAgain(packet)
                return (True, packet)
        return (False, None)
            
    
    def sendPacket(self, packet):
        if (PACKET_SIZE * self.numPacketsSent < self.bitsToSend):
            self.numPacketsSent += 1
            self.outstandingPackets[packet.packetID] = True
            self.timePacketWasSent[packet.packetID] = now()
            if self.link.active is False:
                reactivate(self.link)
                self.link.active = True
            self.link.queue.append(packet)
            # create the timer for this packet
            t = Timer(packet, self.timeout, self)
            activate(t, t.run())
    
    def sendPacketAgain(self, packet):
        self.timePacketWasSent[packet.packetID] = now()
        if self.link.active is False:
            reactivate(self.link)
            self.link.active = True
        self.link.queue.append(packet)
        
    def receivePacket(self, packet):
        if packet.isAck:
            # Count Acks
            if not packet.packetID in self.acks:
                self.acks[packet.packetID] = 0
            self.acks[packet.packetID] += 1
            
            # Remove the outstanding packets from the list
            if packet.packetID in self.outstandingPackets:
                for key in self.outstandingPackets.keys():
                    if key <= packet.packetID:
                        del self.outstandingPackets[key]
                        
            # Update window size
            if not self.enabledCCA:
                self.windowSize += 1
            elif self.congControlAlg == 'AIMD' and self.fastRecovery == False:            
                self.windowSize += 1/float(math.floor(self.windowSize))
            elif self.congControlAlg == 'VEGAS' and self.fastRecovery == False:
                packetRttTime = now() - self.timePacketWasSent[packet.packetID]
                # Set rttMin if has not been set yet
                if self.rttMin == 0:
                    self.rttMin = packetRttTime
                # Update packetRttsToTrack
                self.packetRttsToTrack.append(packetRttTime)
                if len(self.packetRttsToTrack) > self.numPacketsToTrack:
                    self.packetRttsToTrack.popleft()
                self.rtt = sum(self.packetRttsToTrack) / len(self.packetRttsToTrack)
                # Update window size accordingly
                if (self.windowSize / self.rttMin - self.windowSize / self.rtt) < self.alpha:
                    self.windowSize += 1
                else:
                    self.windowSize -= 1
            # Get out fast recovery once missing packet was received
            elif self.fastRecovery and self.mostRecentAck < packet.packetID:
                self.fastRecovery = False
                self.windowSize = self.ssthresh
                #self.windowSize = math.ceil(self.windowSize / 2)
            # Duplicate Ack
            elif self.fastRecovery and self.mostRecentAck == packet.packetID:
                self.windowSize += 1/float(math.floor(self.windowSize))
            else:
                print "Error: Congestion control algorithm not found."
                assert(False)
                
            # Update most recent Ack
            self.mostRecentAck = packet.packetID
            
            # Enable either AIMD or Vegas if windowSize > ssthresh
            if self.windowSize > self.ssthresh:
                self.enabledCCA = True
                        
            # Collecting Stats
            self.windowSizeMonitor.observe(self.windowSize)
            if len(self.outstandingPackets) == 0 and (PACKET_SIZE * self.numPacketsSent >= self.bitsToSend):
                global flowsDone, numFlows
                flowsDone += 1
    
    # Get the min packetID with at least three dup Acks
    def getPacketIdToRetransmit(self):
        minId = sys.maxint
        for key in self.acks.keys():
            if key <= minId and self.acks[key] > 3:
                minId = key
        if minId == sys.maxint:
            return -1
        return minId + 1
       
       
# Timer
# this is called every time a packet is sent. it waits the timeout
# set by the source, then resends packet while it is in list of outst
# packets. once it is no longer in that list, just passivate
class Timer(Process):
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
                global THRESHOLD
                # When time out, update variables to slow start
                self.source.enabledCCA = False
                self.source.fastRecovery = False
                #THRESHOLD = THRESHOLD / 2
                self.source.ssthresh = self.source.sstresh / 2
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
                           packet.desID, packet.isRouterMesg, packet.isAck, message)
        activate(newPacket, newPacket.run())
        return newPacket



# MAIN STARTS HERE

initialize()

# The input parameters: 
#
# nodes[][] is a 2-way array with nodes[i] being the list [isMonitored,isSource,isDest,sourceID,destID,numbits,congID,starttime] if the node is a source
# or destination, and [0,0,0] otherwise if it is a router.
#
# topology[][][] is a 3-way array with topology[i][j] being the list [isMonitored, rate, propTime, bufferCapacity] for the link between nodes i and j,
# and [-1] if one does not exist.  
#
# Everything is on the scale of bits and milliseconds.
#
# Common test cases are the following:
# Test Case 1:
#    nodes = [[1, 1, 0, 0, 1, 160000000, 0, 0], [1, 0, 1, 0, 1, 0, 0, 0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
#    topology = [ [[-1],[-1],[0,10000,10,64],[-1],[-1],[-1]], 
#                 [[-1],[-1],[-1],[-1],[-1],[0,10000,10,64]], 
#                 [[0, 10000, 10, 64],[-1],[-1],[1, 10000, 10, 64],[1, 10000, 10, 64],[-1]],
#                 [[-1],[-1],[1, 10000, 10, 64],[-1],[-1],[0, 10000, 10, 64]], 
#                 [[-1],[-1],[1, 10000, 10, 64],[-1],[-1],[0, 10000, 10, 64]], 
#                 [[-1],[0, 10000, 10, 64],[-1],[0, 10000, 10, 64],[0, 10000, 10, 64],[-1]] ]
#
# Test Case 2:
#    nodes = [[1,1,0,0,1,80000000,0,0],[1,0,1,0,1,0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[1,1,0,6,7,40000000,0,2000],[1,0,1,6,7,0,0,0],[1,1,0,8,9,40000000,0,4000],[1,0,1,8,9,0,0,0]]
#    topology = [ [[-1],[-1],[0,10000,10,128],[-1],[-1],[-1],[-1],[-1],[-1],[-1]],
#                 [[-1],[-1],[-1],[-1],[-1],[0,10000,10,128],[-1],[-1],[-1],[-1]],
#                 [[0,10000,10,128],[-1],[-1],[1,10000,10,128],[-1],[-1],[0,10000,10,128],[-1],[-1],[-1]],
#                 [[-1],[-1],[1,10000,10,128],[-1],[1,10000,10,128],[-1],[-1],[0,10000,10,128],[-1],[-1]],
#                 [[-1],[-1],[-1],[1,10000,10,128],[-1],[1,10000,10,128],[-1],[-1],[0,10000,10,128],[-1]],
#                 [[-1],[0,10000,10,128],[-1],[-1],[1,10000,10,128],[-1],[-1],[-1],[-1],[0,10000,10,128]],
#                 [[-1],[-1],[0,10000,10,128],[-1],[-1],[-1],[-1],[-1],[-1],[-1]],
#                 [[-1],[-1],[-1],[0,10000,10,128],[-1],[-1],[-1],[-1],[-1],[-1]],
#                 [[-1],[-1],[-1],[-1],[0,10000,10,128],[-1],[-1],[-1],[-1],[-1]],
#                 [[-1],[-1],[-1],[-1],[-1],[0,10000,10,128],[-1],[-1],[-1],[-1]] ]

nodes = [[1,1,0,0,1,80000000,0,0],[1,0,1,0,1,0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[1,1,0,6,7,40000000,0,2000],[1,0,1,6,7,0,0,0],[1,1,0,8,9,40000000,0,4000],[1,0,1,8,9,0,0,0]]
topology = [ [[-1],[-1],[0,10000,10,128],[-1],[-1],[-1],[-1],[-1],[-1],[-1]],
             [[-1],[-1],[-1],[-1],[-1],[0,10000,10,128],[-1],[-1],[-1],[-1]],
             [[0,10000,10,128],[-1],[-1],[1,10000,10,128],[-1],[-1],[0,10000,10,128],[-1],[-1],[-1]],
             [[-1],[-1],[1,10000,10,128],[-1],[1,10000,10,128],[-1],[-1],[0,10000,10,128],[-1],[-1]],
             [[-1],[-1],[-1],[1,10000,10,128],[-1],[1,10000,10,128],[-1],[-1],[0,10000,10,128],[-1]],
             [[-1],[0,10000,10,128],[-1],[-1],[1,10000,10,128],[-1],[-1],[-1],[-1],[0,10000,10,128]],
             [[-1],[-1],[0,10000,10,128],[-1],[-1],[-1],[-1],[-1],[-1],[-1]],
             [[-1],[-1],[-1],[0,10000,10,128],[-1],[-1],[-1],[-1],[-1],[-1]],
             [[-1],[-1],[-1],[-1],[0,10000,10,128],[-1],[-1],[-1],[-1],[-1]],
             [[-1],[-1],[-1],[-1],[-1],[0,10000,10,128],[-1],[-1],[-1],[-1]] ]

# Global variables used to determine when to stop the simulation. 
numFlows = 0
flowsDone = 0

# Global Arrays of the Network Objects. Devices includes all the Sources, Destinations, and Routers, and links includes all the link objects.
devices = []
links = []

# Global Arrays containing the Monitors that observe the relevant data
throughputs = []
sendRates = []
windowSizes = []
packetDelays = []
bufferOccs = []
droppedPackets = []
linkFlowRates = []

# Output is written to files with names such as outputName+"throughputs(n).png"
outputName = 'TestCase2noDynamicV100'

# For each device in the nodes, instantiate the appropriate device with its monitors
for id in range(len(nodes)):
    # If the device is a source ...
    if nodes[id][1]:
        numFlows += 1
        
        # Create the monitors
        sendRateMonitor = Monitor(name = 'Send Rate of Source ' + str(id))
        windowSizeMonitor = Monitor(name = 'Window Size of Source '+str(id))
        
        # Create the object, add it to the list of devices, and activate it
        source = Source(id, nodes[id][4], nodes[id][5], nodes[id][6], nodes[id][7], sendRateMonitor,windowSizeMonitor)
        devices.append(source)
        activate(source, source.run())
        
        # If this node should be monitored, add its monitors to the arrays
        if nodes[id][0]:
            sendRates.append(sendRateMonitor)
            windowSizes.append(windowSizeMonitor)
            
    # If the device is a destination ...        
    elif nodes[id][2]:
        
        # Create the Monitors 
        thru = Monitor(name = 'Throughput to Destination ' + str(id))
        pDelay = Monitor(name = 'Packet Delays of Destination ' + str(id))
        
        # Create the object, add it to the list of devices, and activate it
        dest = Destination(id, nodes[id][3], thru, pDelay)
        devices.append(dest)
        activate(dest, dest.run())
        
        # If this node should be monitored, add its monitors to the arrays
        if nodes[id][0]:
            packetDelays.append(pDelay)
            throughputs.append(thru)

    # Otherwise the device is a router ...
    else:
        
        # Create the object, add it to the list of devices, and activate it
        router = Router(id,topology)
        devices.append(router)
        activate(router, router.run())
                            
# For each link in the topology, instantiate the appropriate link with its monitors
for i in range(len(topology)):
    for j in range(len(topology[i])):
        
        # If there is a link between node i and node j (i.e. topology[i][j] isn't [-1]) ...
        if len(topology[i][j]) > 1:
            
            # Create the monitors
            buffOcc = Monitor(name = 'Buffer Occupancies of the Link From ' + str(i) + ' to ' + str(j))
            dropPacket = Monitor(name = 'Dropped Packets of the Link From ' + str(i) + ' to ' + str(j))
            flowRate = Monitor(name = 'Link Flow Rate of the Link From ' + str(i) + ' to ' + str(j))
            
            # Create the object, add it to the list of links, and activate it
            link = Link(topology[i][j][1], devices[i], devices[j], topology[i][j][2], topology[i][j][3], buffOcc, dropPacket, flowRate)
            links.append(link)            
            activate(link, link.run())
            
            # Tell device i that it is connected to this link object
            devices[i].addLink(link)
            
            # If this link should be monitored, add its monitors to the arrays
            if topology[i][j][0]:
                linkFlowRates.append(flowRate)
                bufferOccs.append(buffOcc)
                droppedPackets.append(dropPacket)
            
# The Main Simulation!               
simulate(until = 25000)

# Plot and save all the appropriate measurements. Output files are named outputName+data+(n).png.
print 'producing graphs...'

# Throughputs
n = 0
for m in throughputs:
    plt.plot(m.tseries(), m.yseries(),'o')
    plt.title(m.name)
    plt.xlabel("Time")
    plt.ylabel("Bits per Second")
    plt.savefig(outputName+"throughput" + str(n) + ".png")
    plt.clf()
    n += 1
    
# Send Rates    
n = 0
for m in sendRates:
    plt.plot(m.tseries(), m.yseries(),'o')
    plt.title(m.name)
    plt.xlabel("Time")
    plt.ylabel("Bits per Second")
    plt.savefig(outputName+"sendRate" + str(n) + ".png")
    plt.clf()
    n += 1

# Packet Delays
n = 0
for m in packetDelays:
    plt.plot(m.tseries(), m.yseries(),'o')
    plt.title(m.name)
    plt.xlabel("Time")
    plt.ylabel("Time")
    plt.savefig(outputName+"packetDelay" + str(n) + ".png")
    plt.clf()
    n += 1

# Buffer Occupancies    
n = 0
for m in bufferOccs:
    plt.plot(m.tseries(), m.yseries(),'o')
    plt.title(m.name)
    plt.xlabel("Time")
    plt.ylabel("Packets")
    plt.savefig(outputName+"bufferOccupancies" + str(n) + ".png")
    plt.clf()
    n += 1

# Dropped Packets    
n = 0
for m in droppedPackets:
    plt.plot(m.tseries(), m.yseries(),'o')
    plt.title(m.name)
    plt.xlabel("Time")
    plt.ylabel("Packets")
    plt.savefig(outputName+"droppedPackets" + str(n) + ".png")
    plt.clf()
    n += 1

# Link Flow Rates    
n = 0
for m in linkFlowRates:
    plt.plot(m.tseries(), m.yseries(),'o')
    plt.title(m.name)
    plt.xlabel("Time")
    plt.ylabel("Bits per Second")
    plt.savefig(outputName+"linkSendRate" + str(n) + ".png")
    plt.clf()
    n += 1

# Window Sizes    
n = 0
for m in windowSizes:
    plt.plot(m.tseries(), m.yseries(),'o')
    plt.title(m.name)
    plt.xlabel("Time")
    plt.ylabel("Packets")
    plt.savefig(outputName+"windowSizes" + str(n) + ".png")
    plt.clf()
    n += 1

print("done")
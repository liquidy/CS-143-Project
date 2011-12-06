from SimPy.Simulation import *
from collections import deque
import math

from Link import *
from Packet import *
from Timers import *
from RoutingTimer import *
from Global import *

####################################################################################
# DEVICE
####################################################################################

#
# Describes a superclass from which Router, Source, and Destination extend.
#
class Device(Process):
    
    # Each Device has an ID.
    def __init__(self, ID, globs):
        Process.__init__(self, name="Device" + str(ID))
        self.ID = ID
        self.globs = globs

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
    def __init__(self, ID, sourceID, throughput, packetDelay, globs):
        Device.__init__(self, ID, globs)        # call Device's constructor
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
                           self.sourceID, False, True, message, self.globs)
        activate(newPacket, newPacket.run())
        return newPacket    

        
# Source
class Source(Device):

    def __init__(self, ID, destinationID, bitsToSend, congControlAlg, startTime, sendRateMonitor, windowSizeMonitor, globs):
        Device.__init__(self, ID, globs)
        self.destinationID = destinationID
        self.bitsToSend = bitsToSend
        self.congControlAlg = self.globs.CONGESTION_CONTROL_ALGORITHM
        self.roundTripTime = 0
        self.windowSize = self.globs.INIT_WINDOW_SIZE
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
        self.timeout = self.globs.ACK_TIMEOUT
        self.enabledCCA = False
        self.alpha = self.globs.DEFAULT_ALPHA
        self.ssthresh = self.globs.THRESHOLD
        
        # Variables for congestion control
        self.dupAcks = 0
        self.mostRecentAck = -1
        self.fastRecovery = False
        self.rttMin = 0 # RTT for first packet
        self.rtt = 0 # Average RTT for the last "numPacketsToTrack" packets
        self.packetRttsToTrack = deque() # Queue tracking the last "numPacketsToTrack" packet RTTs
        self.numPacketsToTrack = self.globs.NUM_PACKETS_TO_TRACK_FOR_RTT  # Number of packets to track in packetRttsToTrack
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
                self.ssthresh = self.windowSize / 2
                self.windowSize = self.ssthresh + 3
                self.dupAcks = 0
                
                # Create a new packet based on the packetID, and resent the packet
                message = "Packet " + str(packetIdToRetransmit) + "'s data goes here!"
                # packetId, timesent, sourceID, destid, isroutermsg, isack, msg
                newPacket = Packet(packetIdToRetransmit, now(), self.ID,
                        self.destinationID, False, False, message, self.globs)
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
                        self.sendRateMonitor.observe(self.globs.PACKET_SIZE*self.numPacketsSent / float(now()))
                
            # If everything has been sent, go to sleep
            elif (self.globs.PACKET_SIZE * self.numPacketsSent >= self.bitsToSend):
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
                    self.sendRateMonitor.observe(self.globs.PACKET_SIZE*self.numPacketsSent / float(now()))
            # nothing to retransmit and cannot send new packets
            else:
                self.active = False
                yield passivate, self
                self.active = True
            
    def createPacket(self):
        message = "Packet " + str(self.currentPacketID) + "'s data goes here!"
        newPacket = Packet(self.currentPacketID, now(), self.ID,
                           self.destinationID, False, False, message, self.globs)
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
                pack = self.toRetransmit.pop(0)
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
        if (self.globs.PACKET_SIZE * self.numPacketsSent < self.bitsToSend):
            self.numPacketsSent += 1
            self.outstandingPackets[packet.packetID] = True
            self.timePacketWasSent[packet.packetID] = now()
            if self.link.active is False:
                reactivate(self.link)
                self.link.active = True
            self.link.queue.append(packet)
            # create the timer for this packet
            t = SourceTimer(packet, self.timeout, self)
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
            if packet.packetID > self.mostRecentAck:
                self.dupAcks = 0
            elif packet.packetID == self.mostRecentAck:
                self.dupAcks += 1
            
            # Remove the outstanding packets from the list
            if packet.packetID in self.outstandingPackets:
                print "Ack " + str(packet.packetID)
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
                if self.rttMin == 0 or packetRttTime < self.rttMin:
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
            # Duplicate Ack
            elif self.fastRecovery and self.mostRecentAck == packet.packetID:
                self.windowSize += 1/float(math.floor(self.windowSize))
            else:
                pass
                
            # Update most recent Ack
            if self.mostRecentAck < packet.packetID:
                self.mostRecentAck = packet.packetID
            
            # Enable either AIMD or Vegas if windowSize > threshold
            if self.windowSize > self.ssthresh:
                self.enabledCCA = True
                        
            # Collecting Stats
            self.windowSizeMonitor.observe(self.windowSize)
            if len(self.outstandingPackets) == 0 and (self.globs.PACKET_SIZE * self.numPacketsSent >= self.bitsToSend):
                self.globs.flowsDone += 1
    
    # Get the min packetID with at least three dup Acks
    def getPacketIdToRetransmit(self):
        if self.dupAcks >= 3:
            return self.mostRecentAck + 1
        return -1

        
################################################################################            
#                                   Router                                     #
################################################################################
#
#   A process representing a router; child class of Device.
#       - forwards Packet processes to the appropriate Link process based on the
#           Packet's destination
#       - periodically sends probe Packets along all outgoing Links to measure
#           Packet delays
#       - periodically sends its collected delay data to other Routers and
#           receives delay data from those Routers, then recalculates its
#           routing table
#
#   Fields:
#       active - boolean flag which is False iff the Router process is passive
#       buffer - the input Packet queue; unlimited capacity
#       delayData - a list of average packet delays which is sent to other
#           routers while rerouting; delayData[i] is the average packet delay
#           to node i
#       delays - a 2D array of measured packet delays on outgoing links;
#           delays[i] is a list of measured packet delays to node i
#       haveAck - a list of boolean flags; haveAck[i] is False iff node i must
#           acknowledge receiving the router's delay data before it can reroute
#       haveData - a list of boolean flags; haveData[i] is False iff the router
#           needs delay data from node i before it can reroute
#       linkMap - linkMap[i] is a Link to node i, or None if no such link exists
#       links - a list of outgoing Link objects
#       network - adjancency matrix of link data; network[i][j] is a 3-tuple
#           (monitored?, link rate, propagation delay) if there is a link from
#           device i to device j and [-1] otherwise
#       networkSize - the number of devices in the network
#       rerouting - boolean flag which is True iff the router is currently
#           collecting data from other routers to recalculate its routing table
#       routingTable - routingTable[i] is the Link over which to forward Packets
#           destined for node i
#       timer - a RoutingTimer process for the Router
#
#   Methods:
#       addLink(self, link) - adds the passed Link to the list of outgoing Links
#       dijkstra(self) - recomputes/updates the routing table and exits the
#           rerouting state
#       initiateReroute(self) - initializes the fields used for dynamic routing
#           and enters the rerouting state
#       mapLinks(self) - populates the router's Link map with its outgoing Links
#       processMessage(self, pkt) - reads and reacts to the passed Packet's message
#       receivePacket(self, pkt) - places the passed Packet in the input queue
#       run(self) - main loop for routing; processes/forwards recieved Packets
#       sendPacket(self, pkt, link) - attempts to forward the passed Packet to the
#           passed Link
#       timerHandler(self) - probes outgoing Links for delays if not rerouting;
#           resends unacknowledged topology Packets if rerouting
#       updateNetwork(self, data) - updates the stored network topology with the
#           passed link-state data
#
class Router(Device):
    
    ##
    # Creates a new Router process with the passed device ID and network topology.
    # Initializes most other fields to appropriate values; the routing table and
    # link map are populated when run() is called, and the list of outgoing Links
    # must be filled by the caller.
    ##
    def __init__(self, id, network, globs):
        Device.__init__(self, id, globs)
        self.active = False
        self.buffer = []
        self.delayData = []
        self.delays = []
        for i in range(len(network)):
            self.delays.append([])
        self.haveAck = []
        self.haveData = []
        self.linkMap = [None] * len(network)
        self.links = []  # to be filled by global initializer
        self.network = network
        self.networkSize = len(self.network)
        self.rerouting = False
        self.routingTable = [None] * self.networkSize
        self.timer = RoutingTimer(self)
    
    ##
    # Appends the passed Link to the list of outgoing links.
    ##
    def addLink(self, link):
        self.links.append(link)
    
    ##
    # Recomputes the routing table based on the stored network topology.
    # Sets rerouting to False upon completion.
    ##
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
        
        # run Dijkstra's algorithm to find shortest paths
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
        
        # trace the shortest paths backwards to find correct routing table entries
        for i in range(len(paths)):
            node = paths[i]
            while node[0] is not self.ID: 
                node = paths[node[0]]
            self.routingTable[i] = self.linkMap[node[2]]
            
        self.rerouting = False
        
    ##
    # Sets rerouting to True.  Initializes haveData and haveAck for rerouting.
    # Calculates delayData, resets measured delays array, updates network accordingly.
    # Broadcasts delayData and tells other routers to begin rerouting.
    ##
    def initiateReroute(self):
        self.rerouting = True
        self.haveData = [False] * self.networkSize
        self.haveData[self.ID] = True
        self.haveAck = [False] * self.networkSize
        self.haveAck[self.ID] = True
        
        # calculate delayData
        self.delayData = []
        for i in range(self.networkSize):
            delays = self.delays[i]
            if delays != []:
                self.delayData.append((self.ID, i, sum(delays)/float(len(delays))))
            self.delays[i] = []
            
        self.updateNetwork(self.delayData)
        
        # broadcast rerouting signal with new delayData
        for i in range(self.networkSize):
            if i is not self.ID:
                if not (nodes[i][1] or nodes[i][2]): # if device i is a router
                    pkt = Packet("Delay Data at %d" % (self.ID), now(), self.ID, i, True, False, ("reroute", self.delayData), self.globs)
                    activate(pkt, pkt.run())
                    self.sendPacket(pkt, self.routingTable[i])
                else: # device is not a router, so don't need to communicate with it
                    self.haveData[i] = True
                    self.haveAck[i] = True
                
    ##
    # Populates linkMap with the outgoing Links.
    ##
    def mapLinks(self):
        for link in self.links:
            self.linkMap[link.end.ID] = link
    
    ##
    # Reads the passed Packet's message (type, data) and acts based upon type:
    #   type "probe" - if not rerouting, calculate the delay experienced by the
    #       probe and send this data back to the Packet's source as a probeAck
    #   type "probeAck" - if not rerouting, add data to the array of measured
    #       packet delays
    #   type "reroute" - if not already rerouting, begin rerouting; then process
    #       this Packet as if it had type "topology"
    #   type "topology" - acknowledge receipt of the source's delayData; then if
    #       currently rerouting use data to update the stored network topology,
    #       and if this router has received current delayData from all other
    #       routers and all other routers have acknowledged receipt of this
    #       router's delayData, recompute the routing table and stop rerouting
    #   type "topAck" - if currently rerouting use data to update the stored
    #       network topology, and if this router has received current delayData
    #       from all other routers and all other routers have acknowledged
    #       receipt of this router's delayData, recompute the routing table and
    #       stop rerouting
    ##
    def processMessage(self, pkt):
        source = pkt.sourceID
        (type, data) = pkt.myMessage
        if type == "probe" and not self.rerouting:
            # no data; send probeAck to source with packet delay
            msg = ("probeAck", float(now()) - pkt.timeSent)
            ack = Packet("Probe %d %d" % (self.ID, source), now(), self.ID, source, True, True, msg, self.globs)
            activate(ack, ack.run())
            self.sendPacket(ack, self.routingTable[source])
        elif type == "probeAck" and not self.rerouting:
            # data is packet delay over link to source
            self.delays[source].append(data)
            if len(self.delays[source]) >= self.globs.PROBE_SAMPLE_SIZE:
                self.initiateReroute()
        elif type == "reroute":
            # data is list of (start, end, delay) tuples;
            # if not rerouting, begin rerouting and broadcast delays;
            # send topAck to source;
            # process the recieved data and check if done rerouting
            if self.rerouting == False:
                self.initiateReroute()
            pkt = Packet("Delay Data at %d" % (self.ID), now(), self.ID, source, True, True, ("topAck", self.delayData), self.globs)
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
            pkt = Packet("Delay Data at %d" % (self.ID), now(), self.ID, source, True, True, ("topAck", self.delayData), self.globs)
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
    
    ##
    # Places the passed Packet in the router's input queue.
    ##
    def receivePacket(self, pkt):
            self.buffer.append(pkt)
    
    ##
    # Main loop for Router process.  Initializes linkMap and routingTable before
    # entering the loop.  Loops popping received packets and either forwarding
    # them to their destination, or if they are for this router reading and
    # reacting to their message.  Passivates when the input queue is empty.
    ##
    def run(self):
        self.active = True
        if self.globs.DYNAMIC_ROUTING:
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
            
    ##
    # Enqueues the passed Packet at the passed Link if the Link is not full;
    # drops the Packet otherwise.  If the Link was inactive, reactivates it.
    ##
    def sendPacket(self, pkt, link):
        if len(link.queue) < link.bufferCapacity:
            if not link.active:
                reactivate(link)
                link.active = True
            link.queue.append(pkt)
        else:
            link.droppedPackets += 1
            
    ##
    # Called periodically by a RoutingTimer process if DYNAMIC_ROUTING is True.
    # If not rerouting, sends a delay probe Packet across all outgoing Links.
    # If rerouting, sends delayData to others routers which have not acknowledged
    # this router's delayData.
    ##
    def timerHandler(self):
        if not self.rerouting:
            for link in self.links:
                dst = link.end.ID
                if len(link.queue) >= link.bufferCapacity: # if the probe will be dropped
                    self.delays[dst].append(self.globs.PROBE_DROP_DELAY)
                pkt = Packet("Probe %d %d" % (self.ID, dst), now(), self.ID, dst, True, False, ("probe", None), self.globs)
                activate(pkt, pkt.run())
                self.sendPacket(pkt, link)
        else:
            for i in range(self.networkSize):
                if not self.haveAck[i]:
                    pkt = Packet("Delay Data at %d" % (self.ID), now(), self.ID, i, True, False, ("topology", self.delayData), self.globs)
                    activate(pkt, pkt.run())
                    self.sendPacket(pkt, self.routingTable[i])
    
    ##
    # Updates the stored network topology with the passed link delay data.
    # data is a list of 3-tuples (start, end, delay) of link states.
    ##
    def updateNetwork(self, data):
        for datum in data:
            (start, end, delay) = datum
            self.network[start][end][2] = delay

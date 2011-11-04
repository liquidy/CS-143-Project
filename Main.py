from SimPy.Simulation import * 
import numpy as py
import matplotlib.pyplot as plt

PACKET_SIZE = 8000

class Device(Process):

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

class Destination(Device):

    def __init__(self, ID, sourceID, throughput, packetDelay):
        Device.__init__(self, ID) # call Device
        self.sourceID = sourceID
        self.numPacketsReceived = 0
        self.totalPacketDelay = 0   # calculate the sum of the total packet delays
        self.link = None
        self.active = False
        self.receivedPackets = []  # stores all the rec'd packets
        self.currentAckPacketID = 0
        self.throughput = throughput
        self.packetDelay = packetDelay
        self.packet = None

    def addLink(self, link):
        self.link = link
        
    # process packets as they arrive
    def run(self):
        while True:
            # passivate until receivePacket is called
            self.link.buffMonitor.observe(len(self.link.queue))
            self.link.dropMonitor.observe(self.link.droppedPackets)
            self.active = False
            yield passivate, self
            self.active = True
            
            assert(self.packet is not None)
            
            receivedTime = now()
            self.receivedPackets.append((receivedTime, self.packet))
            self.numPacketsReceived += 1
            
            currDelay = receivedTime - self.packet.timeSent
            self.totalPacketDelay += currDelay
            if not now() == 0:
                self.packetDelay.observe(self.totalPacketDelay / float(self.numPacketsReceived))
                self.throughput.observe(self.numPacketsReceived * self.packet.size / float(now()))
                
            self.acknowledge(self.packet)
            
            print 'Received packet: ' + str(self.packet.packetID) + ' at time ' + str(receivedTime)
            
    # called by link when it is time for packet to get to dest
    def receivePacket(self, packet):
        self.packet = packet
    
    # sends ack once it receives packet
    def acknowledge(self, packet):
        ack = self.createAckPacket()

        # send the ack packet (append it to this dest's link's queue)
        if not self.link.active:
            reactivate(self.link)
            self.link.active = True
        self.link.queue.append(ack)
        
    def createAckPacket(self):
        #(packetID, timeSent, sourceID, desID, isRouterMesg, isAck, myMessage, monitor)
        message = "Acknowledgement packet " + str(self.currentAckPacketID) + "'s data goes here!"

        # need to instantiate monitor object to pass to ack packet
        newPacket = Packet(self.currentAckPacketID, now(), self.ID, 
                           self.packet.sourceID, False, True, message)

        self.currentAckPacketID += 1
        activate(newPacket, newPacket.run())
        return newPacket  

class Link(Process):

    def __init__(self, linkRate, start, end, propTime, buffMonitor, dropMonitor, flowMonitor):
        Process.__init__(self)
        self.queue = []
        self.linkRate = linkRate
        self.start = start
        self.end = end
        self.propTime = propTime
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
            print "t = %.2f: Job-%d departs" % (now(), packet.packetID)        

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

class Router(Device):
    
    # network - adjancency matrix of (monitored, link rate, propagation delay)
    def __init__(self, id, network, cap):
        Device.__init__(self, id)
        self.network = network
        self.bufferCapacity = cap
        self.buffer = []
        self.active = False
        # self.rerouting = False
        self.links = []  # to be filled by global initializer
        self.linkMap = [None] * len(self.network) # linkMap[i] is a link to node i
        # self.delays = []
        self.routingTable = [None] * len(self.network)
    
    def run(self):
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
            link = self.routingTable[dest]
            self.sendPacket(pkt, link)
            # TODO: check for router messages
            
    def sendPacket(self, pkt, link):
        if len(link.queue) < self.bufferCapacity:
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
        for i in range(len(self.network)):
            paths.append((None, None, i))
        paths[self.ID] = (self.ID, 0, self.ID)
        # initialize a list of IDs of undiscovered nodes
        left = []
        for i in range(len(self.network)):
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
            for i in range(len(self.network)):
                link = self.network[nextNode][i]
                if len(link) == 3: # if a link exists between nextNode and node i
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

class Source(Device):

    def __init__(self, ID, destinationID, flowRate, bitsToSend, congestionAlgID, monitor):
        Device.__init__(self, ID)
        self.destinationID = destinationID
        self.bitsToSend = bitsToSend
        self.flowRate = flowRate
        self.congestionAlgID = congestionAlgID
        self.roundTripTime = 0
        self.windowSize = 10000000
        self.currentPacketID = 0
        self.outstandingPackets = {}
        self.link = None
        self.active = False
        self.sendRateMonitor = monitor
        self.numPacketsSent = 0

    def addLink(self, link):
        self.link = link

    # TODO:  Create new timer process.  Source creates timer for each packet
    # and passes it the packet's ID.  When the timer expires, if the packet has
    # not been acknowledged, it creates a new packet with the same ID and sends
    # it from the Source before resetting its time.  
    #(secretly a congestion control process)
    def run(self):
        self.active = True
        while PACKET_SIZE * self.numPacketsSent < self.bitsToSend:
            if len(self.outstandingPackets) >= self.windowSize:                
                self.active = False                
                yield passivate, self
                self.active = True
            if not self.link.active:
                reactivate(self.link)
                self.link.active = True
            packet = self.createPacket()
            self.sendPacket(packet)
            self.link.buffMonitor.observe(len(self.link.queue))
            self.link.dropMonitor.observe(self.link.droppedPackets)
            yield hold, self, packet.size/float(self.link.linkRate)
            if not now() == 0:
                self.sendRateMonitor.observe(PACKET_SIZE*self.numPacketsSent / float(now()))
            
    def createPacket(self):
        message = "Packet " + str(self.currentPacketID) + "'s data goes here!"
        newPacket = Packet(self.currentPacketID, now(), self.ID, 
                           self.destinationID, False, False, message)
        self.currentPacketID += 1
        activate(newPacket, newPacket.run())
        return newPacket
    
    def sendPacket(self, packet):
        self.numPacketsSent += 1
        self.outstandingPackets[packet.packetID] = True
        self.link.queue.append(packet)
        
    def receivePacket(self, packet):
        if packet.isAck:
            del self.outstandingPackets[packet.packetID]
            
# MAIN STARTS HERE

# main takes the following arguments:
# topology[][][], a 3-way array with topology[i][j] being the list [monitored, rate, propogation delay] for the link between nodes i and j, 
#   and -1 if one does not exist
# nodes[][], a 2-way array with nodes[i] being the list 
#   [monitored, bufferCapacity, isSource, isDest, sourceID, destID, flowrate, numbits, congID, starttime]

initialize()
topology = [[[-1],[-1],[1, 10000, 15]], [[-1],[-1],[1, 10000, 15]], [[1,10000,15],[1,10000,15],[-1]]]
nodes = [[1, 64000, 1, 0, 0, 1, 500, 10000000, 0, 0], [1, 64000, 0, 1, 0, 1, 500, 0, 0, 0],[0,64,0,0]]
devices = []
links = []
throughputs = []
sendRates = []
packetDelays = []
bufferOccs = []
droppedPackets = []
linkFlowRates = []
#For each device in the nodes, instantiate the appropriate device with its monitors
for id in range(len(nodes)):
    if nodes[id][2]:
        m = Monitor(name = 'Send Rate of Source ' + str(id))
        devices.append(Source(id, nodes[id][5], nodes[id][6], nodes[id][7], nodes[id][8], m))
        activate(devices[id], devices[id].run(), at=nodes[id][9])
        if nodes[id][0]:
            sendRates.append(m)
    elif nodes[id][3]:
        thru = Monitor(name = 'Throughput to Destination ' + str(id))
        throughputs.append(thru)
        pDelay = Monitor(name = 'Packet Delays of Destination ' + str(id))              
        devices.append(Destination(id, nodes[id][5], thru, pDelay))
        activate(devices[id], devices[id].run())
        if nodes[id][0]:
            packetDelays.append(pDelay)
    else:                  
        devices.append(Router(id, topology, nodes[id][1]))
        activate(devices[id], devices[id].run())       
                            
#For each link in the topology, instantiate the appropriate link with its monitors
for i in range(len(topology)):
    for j in range(len(topology[i])):
        if len(topology[i][j]) > 1:
            if topology[i][j][0]:
                buffOcc = Monitor(name = 'Buffer Occupancies of the Link From '  + str(i) + ' to ' + str(j))
                dropPacket = Monitor(name = 'Dropped Packets of the Link From '  + str(i) + ' to ' + str(j))  
                flowRate = Monitor(name = 'Link Flow Rate of the Link From '  + str(i) + ' to ' + str(j))
                linkFlowRates.append(flowRate)
                bufferOccs.append(buffOcc)
                droppedPackets.append(dropPacket)
            link = Link(topology[i][j][1], devices[i], devices[j], topology[i][j][2], buffOcc, dropPacket, flowRate)
            activate(link, link.run())
            links.append(link)
            devices[i].addLink(link)
                
simulate(until=1000000000)

### Plot and save all the measurements. 
for m in throughputs:
    plt.plot(m.tseries(), m.yseries(),'o')
    plt.title(m.name)
    plt.xlabel("Time")
    plt.ylabel("Bits per Second")
    plt.savefig("top2throughput" + str(i) + ".png")
    plt.clf()
    
for m in sendRates:
    plt.plot(m.tseries(), m.yseries(),'o')
    plt.title(m.name)
    plt.xlabel("Time")
    plt.ylabel("Bits per Second")
    plt.savefig("top2sendRate" + str(i) + ".png")
    plt.clf()
    
for m in packetDelays:
    plt.plot(m.tseries(), m.yseries(),'o')
    plt.title(m.name)
    plt.xlabel("Time")
    plt.ylabel("Time")
    plt.savefig("top2packetDelay" + str(i) + ".png")
    plt.clf()
    
for m in bufferOccs:
    plt.plot(m.tseries(), m.yseries(),'o')
    plt.title(m.name)
    plt.xlabel("Time")
    plt.ylabel("Packets")
    plt.savefig("top2bufferOccupancies" + str(i) + ".png")
    plt.clf()
    
for m in droppedPackets:
    plt.plot(m.tseries(), m.yseries(),'o')
    plt.title(m.name)
    plt.xlabel("Time")
    plt.ylabel("Packets")
    plt.savefig("top2droppedPackets" + str(i) + ".png")
    plt.clf()
    
for m in linkFlowRates:
    plt.plot(m.tseries(), m.yseries(),'o')
    plt.title(m.name)
    plt.xlabel("Time")
    plt.ylabel("Bits per Second")
    plt.savefig("top2linkSendRate" + str(i)  + ".png")
    plt.clf()
print("done")
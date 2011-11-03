from SimPy.Simulation import *
import numpy as py
import matplotlib.pyplot as plt

class Device(Process):

    def __init__(self, ID):
        Process.__init__(self, name="Device"+str(ID))
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

    def __init__(self, ID, sourceID,throughput,packetDelay):
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


    def addLink(self, link):
        self.link = link
        
        
    # process packets as they arrive
    def run(self):
        while True:
            # passivate until receivePacket is called
            yield passivate, self
            
            
    # called by link when it is time for packet to get to dest
    def receivePacket(self, packet):
        receivedTime = now()
        self.receivedPackets.append((receivedTime, packet)) # list holds (time, packet) tuples
        self.numPacketsReceived += 1
        
        # calculate the delay
        currDelay = receivedTime - packet.timeSent
        self.totalPacketDelay += currDelay
        if not now() == 0:
            self.packetDelay.observe(self.totalPacketDelay/now())
            self.throughput.observe(self.numPacketsReceived*packet.size/now())
        
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

class Router(Device):
    
    # network - adjancency matrix of (monitored, link rate, propagation delay)
    def __init__(self, id, network, cap, bufferMonitor, dropMonitor):
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
        self.bufferMonitor = bufferMonitor
        self.dropMonitor = dropMonitor
    
    def run(self):
        self.mapLinks()
        self.dijkstra()
        while True:
            if len(self.buffer) == []:
                self.active = False
                yield passivate, self
            self.active = True
            pkt = self.buffer.pop(0)
            dest = pkt.desID
            link = self.routingTable(desID)
            self.sendPacket(pkt, link)
            # TODO: check for router messages
            
    def sendPacket(pkt, link):
        if not link.active:
            reactivate(link)
            link.active = True
        if len(link.queue) < self.bufferCapacity:
            link.queue.append(pkt)
            
    def receivePacket(self, pkt):
            self.buffer.append(pkt)
    
    def addLink(self, link):
        self.links.append(link)
        
    def mapLinks(self):
        for link in self.links:
            self.linkMap[link.end] = link
    
    def dijkstra(self):
        # initialize a list of [predecessor, distance] pairs
        paths = [[None, None]] * len(self.network)
        paths[self.id] = [self.id, 0]
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
                    if dist < paths[i][1]: # if we found a shorter path to node i
                        paths[i][0] = nextNode
                        paths[i][1] = dist
        
        for i in range(len(paths)):
            node = paths[i]
            # find next step toward node i from self
            while node[0] is not self.id: 
                node = paths[node[0]]
            # update routing table
            self.routingTable[i] = self.linkMap[paths.index(node)]

class Source(Device):

    def __init__(self, ID, destinationID, flowRate, bitsToSend, congestionAlgID,m):
        Device.__init__(self, ID)
        self.destinationID = destinationID
        self.bitsToSend = bitsToSend
        self.flowRate = flowRate
        self.congestionAlgID = congestionAlgID
        self.roundTripTime = 0
        self.windowSize = 100
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
            if len(self.outstandingPackets) >= self.windowSize:
                yield passivate, self
            if not self.link.active:
                reactivate(self.link)
                self.link.active = True
            packet = self.createPacket()
            self.sendPacket(packet)
            self.numPacketsSent = self.numPacketsSent + 1
            if not now() == 0:
                self.sendRateMonitor.observe(self.numPacketsSent/now())
            self.outstandingPackets[packet] = True;
            
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

class Link(Process):

    def __init__(self, linkRate, start, end, propTime, monitor):
        Process.__init__(self)
        self.queue = []
        self.linkRate = linkRate
        self.start = start
        self.end = end
        self.propTime = propTime
        self.monitor = monitor
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
            packet = self.queue.pop(0)
            # wait for trans time
            yield hold, self, packet.size/self.linkRate
            self.packetsSent = self.packetsSent + 1
            if not now() == 0:
                self.monitor.observe(packet.size*self.packetsSent/now())
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
                reactivate(self.device)
                self.device.busy = True
            self.device.receivePacket(self)        

# main takes the following arguments:
# topology[][][], a 3-way array with topology[i][j] being the list [monitored, rate, propogation delay] for the link between nodes i and j,
#   and -1 if one does not exist
# nodes[][], a 2-way array with nodes[i] being the list 
#   [monitored, bufferCapacity, isSource, isDest, sourceID, destID, flowrate, numbits, congID, starttime]

initialize()
topology = [[[-1],[1,10000,15]],[[1,10000,15],[-1]]]
nodes = [[1,64000,1,0,0,1,500,10000000,0,0],[1,64000,0,1,0,1,500,0,0,0]]
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
        devices.append(Source(id,nodes[id][5],nodes[id][6],nodes[id][7],nodes[id][8],m))
        activate(devices[id],devices[id].run(),at=nodes[id][9])
        if nodes[id][0]:
            sendRates.append(Monitor(name = 'Send Rate of Source ' + str(id)))
    elif nodes[id][3]:
        thru = Monitor(name = 'Throughput to Destination ' + str(id))
        throughputs.append(thru)
        pDelay = Monitor(name = 'Packet Delays of Destination ' + str(id))              
        devices.append(Destination(id, nodes[id][5],thru,pDelay))
        activate(devices[id],devices[id].run())
        if nodes[id][0]:
            packetDelays.append(pDelay)
    else:
        buffOcc = Monitor(name = 'Buffer Occupancies of Router ' +str(id))
        dropPacket = Monitor(name = 'Dropped Packets of Router ' + str(id))                    
        devices.append(Router(id,topology,nodes[id][1],buffOcc,dropPacket))
        activate(devices[id],devices[id].run())
        if nodes[id][0]:
            bufferOccs.append(Monitor(buffOcc))
            droppedPackets.append(dropPacket)
        
                            
#For each link in the topology, instantiate the appropriate link with its monitors
for i in range(len(topology)):
    for j in range(len(topology[i])):
        if len(topology[i][j]) > 1:
            if topology[i][j][0]:
                flowRate = Monitor(name = 'Link Flow Rate of the Link Between Devices ' + str(i) + ' and ' + str(j))
                linkFlowRates.append(flowRate)
            link = Link(topology[i][j][1],devices[i],devices[j],topology[i][j][2],flowRate)
            activate(link,link.run())
            links.append(link)
            devices[i].addLink(link)
                
simulate(until=1000000000)

# Plot and save all the measurements. 
for m in throughputs:
    plt.plot(m.tseries(),m.yseries())
    plt.title(m.name)
    plt.xlabel("Time")
    plt.ylabel("Bits per Second")
    plt.savefig("throughput"+str(i)+".png")
    
for m in sendRates:
    plt.plot(m.tseries(),m.yseries())
    plt.title(m.name)
    plt.xlabel("Time")
    plt.ylabel("Bits per Second")
    plt.savefig("sendRate"+str(i)+".png")
    
for m in packetDelays:
    plt.plot(m.tseries(),m.yseries())
    plt.title(m.name)
    plt.xlabel("Time")
    plt.ylabel("Time")
    plt.savefig("packetDelay"+str(i)+".png")
    
for m in bufferOccs:
    plt.plot(m.tseries(),m.yseries())
    plt.title(m.name)
    plt.xlabel("Time")
    plt.ylabel("Packets")
    plt.savefig("bufferOccupancies"+str(i)+".png")
    
for m in droppedPackets:
    plt.plot(m.tseries(),m.yseries())
    plt.title(m.name)
    plt.xlabel("Time")
    plt.ylabel("Packets")
    plt.savefig("droppedPackets"+str(i)+".png")
    
for m in linkFlowRates:
    plt.plot(m.tseries(),m.yseries())
    plt.title(m.name)
    plt.xlabel("Time")
    plt.ylabel("Bits per Second")
    plt.savefig("linkSendRate" + str(i) +".png")
print("done")
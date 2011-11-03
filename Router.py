from SimPy.Simulation import *

class Router(Device):
    
    # network - adjancency matrix of (monitored, link rate, propagation delay)
    def __init__(self, id, network, cap, bufferMonitor, dropMonitor):
        Device.__init__(self, id)
        self.network = network
        self.bufferCapacity = cap
        self.buffer = []
        self.busy = False
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
                self.busy = False
                yield passivate, self
            self.busy = True
            pkt = self.buffer.pop(0)
            dest = pkt.desID
            link = self.routingTable(desID)
            self.sendPacket(pkt, link)
            # TODO: check for router messages
            
    def sendPacket(pkt, link):
        if not link.active:
            reactivate(link)
        link.queue.append(pkt)
            
    def receivePacket(self, pkt):
        if len(self.buffer) < bufferCapacity:
            self.buffer.append(pkt)
        else: # drop the packet
            pkt.cancel()
    
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
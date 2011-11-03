from SimPy.Simulation import *
import numpy as py
import matplotlib.pyplot as plt

# main takes the following arguments:
# topology[][][], a 3-way array with topology[i][j] being the list [monitored, rate, propogation delay] for the link between nodes i and j,
#   and -1 if one does not exist
# nodes[][], a 2-way array with nodes[i] being the list 
#   [monitored, bufferCapacity, isSource, isDest, sourceID, destID, flowrate, numbits, congID, starttime]

class Main:
    def main(self,topology,nodes):
        initialize()
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
            if nodes[id][3]:
                m = Monitor(name = 'Send Rate of Source %n' % id)
                devices[id] = Source(id,nodes[id][6],nodes[id][7],nodes[id][8],nodes[id][9],m)
                activate(devices[id],devices[id].run(),at=nodes[id][10])
                if nodes[id][1]:
                    sendRates.append(Monitor(name = 'Send Rate of Source %n' % id))
            elif nodes[id][4]:
                thru = Monitor(name = 'Throughput to Destination %n' % id)
                throughputs.append(thru)
                pDelay = Monitor(name = 'Packet Delays of Destination %n' % id)              
                devices[id] = Destination(id, nodes[id][6],thru,pDelay)
                activate(devices[id],devices[id].run())
                if nodes[id][1]:
                    packetDelays.append(pDelay)
            else:
                buffOcc = Monitor(name = 'Buffer Occupancies of Router %n' % id)
                dropPacket = Monitor(name = 'Dropped Packets of Router %n' % id)                    
                devices[id] = Router(id,topology,nodes[id][2],buffOcc,dropPacket)
                activate(devices[id],devices[id].run())
                if nodes[id][1]:
                    bufferOccs.append(Monitor(buffOcc))
                    droppedPackets.append(dropPacket)
                
                                    
        #For each link in the topology, instantiate the appropriate link with its monitors
        for i in range(len(topology)):
            for j in range(len(topology[i])):
                if len(topology[i][j]) > 1:
                    if topology[i][j][1]:
                        flowRate = Monitor(name = 'Link Flow Rate of the Link Between Devices %n and %n' % i % j)
                        linkFlowRates.append(flowRate)
                    link = Link(topology[i][j][2],devices[i],devices[j],topology[i][j][3],flowRate)
                    activate(link,link.run())
                    links.append(link)
                    devices[i].addLink(flink)
                        
        simulate()
        
        # Plot and save all the measurements. 
        for m in throughputs:
            plt.plot(m.tseries(),m.yseries())
            plt.title(m.name)
            plt.xlabel("Time")
            plt.ylabel("Bits per Second")
            plt.savefig("thrughput%n.png" % i)
            
        for m in sendRates:
            plt.plot(m.tseries(),m.yseries())
            plt.title(m.name)
            plt.xlabel("Time")
            plt.ylabel("Bits per Second")
            plt.savefig("sendRate%n.png" % i)
            
        for m in packetDelays:
            plt.plot(m.tseries(),m.yseries())
            plt.title(m.name)
            plt.xlabel("Time")
            plt.ylabel("Time")
            plt.savefig("packetDelay%n.png" % i)
            
        for m in bufferOccs:
            plt.plot(m.tseries(),m.yseries())
            plt.title(m.name)
            plt.xlabel("Time")
            plt.ylabel("Packets")
            plt.savefig("bufferOccupancies%n.png" % i)
            
        for m in droppedPackets:
            plt.plot(m.tseries(),m.yseries())
            plt.title(m.name)
            plt.xlabel("Time")
            plt.ylabel("Packets")
            plt.savefig("droppedPackets%n.png" % i)
            
        for m in linkFlowRates:
            plt.plot(m.tseries(),m.yseries())
            plt.title(m.name)
            plt.xlabel("Time")
            plt.ylabel("Bits per Second")
            plt.savefig("linkSendRate%n.png" % i)
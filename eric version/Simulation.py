from SimPy.Simulation import *
from collections import deque
import numpy as py
import matplotlib.pyplot as plt
import math
import sys
from Devices import *
from Link import *
from Global import *

class Simulation():

    def __init__(self):
        self.globs = Global()
    
    def run(self):
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
        
        if self.globs.TEST_CASE == 1:
            nodes = [[1, 1, 0, 0, 1, 160000000, 0, 0], [1, 0, 1, 0, 1, 0, 0, 0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
            topology = [ [[-1],[-1],[0,15000,10,64],[-1],[-1],[-1]], 
                    [[-1],[-1],[-1],[-1],[-1],[0,10000,10,64]], 
                    [[0, 15000, 10, 64],[-1],[-1],[1, 10000, 10, 64],[1, 10000, 10, 64],[-1]],
                    [[-1],[-1],[1, 10000, 10, 64],[-1],[-1],[0, 10000, 10, 64]], 
                    [[-1],[-1],[1, 10000, 10, 64],[-1],[-1],[0, 10000, 10, 64]], 
                    [[-1],[0, 10000, 10, 64],[-1],[0, 10000, 10, 64],[0, 10000, 10, 64],[-1]] ]
        elif self.globs.TEST_CASE == 2:
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
        else:
            nodes = self.globs.TEST_CASE[0]
            topology = self.globs.TEST_CASE[1]
        
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
        outputName = 'TestCase' + str(self.globs.TEST_CASE)
        if self.globs.DYNAMIC_ROUTING:
            outputName += 'Dynamic'
        outputName += self.globs.CONGESTION_CONTROL_ALGORITHM
        
        # For each device in the nodes, instantiate the appropriate device with its monitors
        for id in range(len(nodes)):
            # If the device is a source ...
            if nodes[id][1]:
                self.globs.numFlows += 1
                
                # Create the monitors
                sendRateMonitor = Monitor(name = 'Send Rate of Source ' + str(id))
                windowSizeMonitor = Monitor(name = 'Window Size of Source '+str(id))
                
                # Create the object, add it to the list of devices, and activate it
                source = Source(id, nodes[id][4], nodes[id][5], nodes[id][6], nodes[id][7], sendRateMonitor, windowSizeMonitor, self.globs)
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
                dest = Destination(id, nodes[id][3], thru, pDelay, self.globs)
                devices.append(dest)
                activate(dest, dest.run())
                
                # If this node should be monitored, add its monitors to the arrays
                if nodes[id][0]:
                    packetDelays.append(pDelay)
                    throughputs.append(thru)
        
            # Otherwise the device is a router ...
            else:
                
                # Create the object, add it to the list of devices, and activate it
                router = Router(id,topology, self.globs)
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
        simulate(until = 50000)
        
        # Plot and save all the appropriate measurements. Output files are named outputName+data+(n).png.
        print 'producing graphs...'
        
        # Window Sizes    
        n = 0
        for m in windowSizes:
            plt.plot(m.tseries(), m.yseries())
            plt.title(m.name)
            plt.xlabel("Time")
            plt.ylabel("Packets")
            plt.savefig(outputName+"windowSizes" + str(n) + ".png")
            plt.clf()
            n += 1
        
        # Throughputs
        n = 0
        for m in throughputs:
            L = len(m.yseries())
            deltay = [a-b for a,b in zip(m.yseries()[2*self.globs.THROUGHPUT_AVERAGE:L],m.yseries()[0:L-2*self.globs.THROUGHPUT_AVERAGE])]
            deltax = [a-b for a,b in zip(m.tseries()[2*self.globs.THROUGHPUT_AVERAGE:L],m.tseries()[0:L-2*self.globs.THROUGHPUT_AVERAGE])]
            thru = [float(a)/b for a,b in zip(deltay,deltax)]
            plt.plot(m.tseries()[self.globs.THROUGHPUT_AVERAGE:L-self.globs.THROUGHPUT_AVERAGE], thru)
            plt.title(m.name)
            plt.xlabel("Time")
            plt.ylabel("Bits per Second")
            plt.savefig(outputName+"throughput" + str(n) + ".png")
            plt.clf()
            n += 1
            
        # Send Rates    
        n = 0
        for m in sendRates:
            plt.plot(m.tseries(), m.yseries())
            plt.title(m.name)
            plt.xlabel("Time")
            plt.ylabel("Bits per Second")
            plt.savefig(outputName+"sendRate" + str(n) + ".png")
            plt.clf()
            n += 1
        
        # Packet Delays
        n = 0
        for m in packetDelays:
            plt.plot(m.tseries(), m.yseries())
            plt.title(m.name)
            plt.xlabel("Time")
            plt.ylabel("Time")
            plt.savefig(outputName+"packetDelay" + str(n) + ".png")
            plt.clf()
            n += 1
        
        # Buffer Occupancies    
        n = 0
        for m in bufferOccs:
            plt.plot(m.tseries(), m.yseries())
            plt.title(m.name)
            plt.xlabel("Time")
            plt.ylabel("Packets")
            plt.savefig(outputName+"bufferOccupancies" + str(n) + ".png")
            plt.clf()
            n += 1
        
        # Dropped Packets    
        n = 0
        for m in droppedPackets:
            plt.plot(m.tseries(), m.yseries())
            plt.title(m.name)
            plt.xlabel("Time")
            plt.ylabel("Packets")
            plt.savefig(outputName+"droppedPackets" + str(n) + ".png")
            plt.clf()
            n += 1
        
        # Link Flow Rates    
        #n = 0
        #for m in linkFlowRates:
        #    plt.plot(m.tseries(), m.yseries(),'o')
        #    plt.title(m.name)
        #    plt.xlabel("Time")
        #    plt.ylabel("Bits per Second")
        #    plt.savefig(outputName+"linkSendRate" + str(n) + ".png")
        #    plt.clf()
        #    n += 1
        
        
        
        print("done")
        
if __name__ == '__main__':
    sim = Simulation()
    sim.run()
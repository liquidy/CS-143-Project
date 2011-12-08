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
        # nodes[][] is a 2-way array with nodes[i] being the list [ID, isSource,isDest,isRouter,isMonitored,sourceID,destID,numbits,starttime] if the node is a source
        # or destination, and [ID,0,0,1] otherwise if it is a router.
        #
        # topology[][][] is a 3-way array with topology[i][j] being the list [isMonitored, rate, propTime, bufferCapacity] for the link between nodes i and j,
        # and [-1] if one does not exist.  
        #
        # Everything is on the scale of bits and milliseconds.
        #
        # Common test cases are the following:
        # Test Case 1:
        #    nodes = [[0,1,0,0,1,0,1,160000000,0], [1,0,1,0,1,0,1],[2,0,0,1],[3,0,0,1],[4,0,0,1],[5,0,0,1]]
        #    topology = [ [[-1],[-1],[0,10000,10,64],[-1],[-1],[-1]], 
        #                 [[-1],[-1],[-1],[-1],[-1],[0,10000,10,64]], 
        #                 [[0, 10000, 10, 64],[-1],[-1],[1, 10000, 10, 64],[1, 10000, 10, 64],[-1]],
        #                 [[-1],[-1],[1, 10000, 10, 64],[-1],[-1],[0, 10000, 10, 64]], 
        #                 [[-1],[-1],[1, 10000, 10, 64],[-1],[-1],[0, 10000, 10, 64]], 
        #                 [[-1],[0, 10000, 10, 64],[-1],[0, 10000, 10, 64],[0, 10000, 10, 64],[-1]] ]
        #
        # Test Case 2:
        #    nodes = [[0,1,0,0,1,0,1,160000000,0],[1,0,1,0,1,0,1],[2,0,0,0,1],[3,0,0,1],[4,0,0,1],[5,0,0,1],[6,1,0,0,1,6,7,100000000,2000],[7,0,1,0,1,6,7],[8,1,0,0,1,8,9,100000000,13000],[9,0,1,0,1,8,9]]
        #    topology = [ [[-1],[-1],[0,20000,10,128],[-1],[-1],[-1],[-1],[-1],[-1],[-1]],
        #                 [[-1],[-1],[-1],[-1],[-1],[0,20000,10,128],[-1],[-1],[-1],[-1]],
        #                 [[0,20000,10,128],[-1],[-1],[1,10000,10,128],[-1],[-1],[0,20000,10,128],[-1],[-1],[-1]],
        #                 [[-1],[-1],[1,10000,10,128],[-1],[1,10000,10,128],[-1],[-1],[0,20000,10,128],[-1],[-1]],
        #                 [[-1],[-1],[-1],[1,10000,10,128],[-1],[1,10000,10,128],[-1],[-1],[0,20000,10,128],[-1]],
        #                 [[-1],[0,20000,10,128],[-1],[-1],[1,10000,10,128],[-1],[-1],[-1],[-1],[0,20000,10,128]],
        #                 [[-1],[-1],[0,20000,10,128],[-1],[-1],[-1],[-1],[-1],[-1],[-1]],
        #                 [[-1],[-1],[-1],[0,20000,10,128],[-1],[-1],[-1],[-1],[-1],[-1]],
        #                 [[-1],[-1],[-1],[-1],[0,20000,10,128],[-1],[-1],[-1],[-1],[-1]],
        #                 [[-1],[-1],[-1],[-1],[-1],[0,20000,10,128],[-1],[-1],[-1],[-1]] ]
        
        if self.globs.TEST_CASE == 1:
            nodes = [[0,1,0,0,1,0,1,160000000,0], [1,0,1,0,1,0,1],[2,0,0,1],[3,0,0,1],[4,0,0,1],[5,0,0,1]]
            topology = [ [[-1],[-1],[0,10000,10,64],[-1],[-1],[-1]], 
                         [[-1],[-1],[-1],[-1],[-1],[0,10000,10,64]], 
                         [[0, 10000, 10, 64],[-1],[-1],[1, 10000, 10, 64],[1, 10000, 10, 64],[-1]],
                         [[-1],[-1],[1, 10000, 10, 64],[-1],[-1],[0, 10000, 10, 64]], 
                         [[-1],[-1],[1, 10000, 10, 64],[-1],[-1],[0, 10000, 10, 64]], 
                         [[-1],[0, 10000, 10, 64],[-1],[0, 10000, 10, 64],[0, 10000, 10, 64],[-1]] ]
                         
        elif self.globs.TEST_CASE == 2:
            nodes = [[0,1,0,0,1,0,1,160000000,0],[1,0,1,0,1,0,1],[2,0,0,0,1],[3,0,0,1],[4,0,0,1],[5,0,0,1],[6,1,0,0,1,6,7,100000000,2000],[7,0,1,0,1,6,7],[8,1,0,0,1,8,9,100000000,13000],[9,0,1,0,1,8,9]]
            topology = [ [[-1],[-1],[0,20000,10,128],[-1],[-1],[-1],[-1],[-1],[-1],[-1]],
                         [[-1],[-1],[-1],[-1],[-1],[0,20000,10,128],[-1],[-1],[-1],[-1]],
                         [[0,20000,10,128],[-1],[-1],[1,10000,10,128],[-1],[-1],[0,20000,10,128],[-1],[-1],[-1]],
                         [[-1],[-1],[1,10000,10,128],[-1],[1,10000,10,128],[-1],[-1],[0,20000,10,128],[-1],[-1]],
                         [[-1],[-1],[-1],[1,10000,10,128],[-1],[1,10000,10,128],[-1],[-1],[0,20000,10,128],[-1]],
                         [[-1],[0,20000,10,128],[-1],[-1],[1,10000,10,128],[-1],[-1],[-1],[-1],[0,20000,10,128]],
                         [[-1],[-1],[0,20000,10,128],[-1],[-1],[-1],[-1],[-1],[-1],[-1]],
                         [[-1],[-1],[-1],[0,20000,10,128],[-1],[-1],[-1],[-1],[-1],[-1]],
                         [[-1],[-1],[-1],[-1],[0,20000,10,128],[-1],[-1],[-1],[-1],[-1]],
                         [[-1],[-1],[-1],[-1],[-1],[0,20000,10,128],[-1],[-1],[-1],[-1]] ]
        else:
            nodes = self.globs.TEST_CASE[0]
            topology = self.globs.TEST_CASE[1]
        self.globs.nodes = nodes
        
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
        
        # Global values for determining when the simulation is over
        self.globs.numFlows = 0
        self.globs.flowsDone = 0
        
        # Output is written to files with names such as outputName+"throughputs(n).png"
        outputName = 'TestCase' + str(self.globs.TEST_CASE)
        if self.globs.DYNAMIC_ROUTING:
            outputName += 'Dynamic'
        outputName += self.globs.CONGESTION_CONTROL_ALGORITHM
        
        # For each device in the nodes, instantiate the appropriate device with its monitors
        for i in range(len(nodes)):
            # If the device is a source ...
            if nodes[i][1]:
                self.globs.numFlows += 1
                
                # Create the monitors
                sendRateMonitor = Monitor(name = 'Send Rate of Source ' + str(nodes[i][0]))
                windowSizeMonitor = Monitor(name = 'Window Size of Source '+str(nodes[i][0]))
                
                # Create the object, add it to the list of devices, and activate it
                source = Source(nodes[i][0], nodes[i][6], nodes[i][7], nodes[i][8], sendRateMonitor, windowSizeMonitor, self.globs)
                devices.append(source)
                activate(source, source.run())
                
                # If this node should be monitored, add its monitors to the arrays
                if nodes[i][4]:
                    sendRates.append(sendRateMonitor)
                    windowSizes.append(windowSizeMonitor)
                    
            # If the device is a destination ...        
            elif nodes[i][2]:
                
                # Create the Monitors 
                thru = Monitor(name = 'Throughput to Destination ' + str(id))
                pDelay = Monitor(name = 'Packet Delays of Destination ' + str(id))
            
                # Create the object, add it to the list of devices, and activate it
                dest = Destination(nodes[i][0], nodes[i][5], thru, pDelay, self.globs)
                devices.append(dest)
                activate(dest, dest.run())
                
                # If this node should be monitored, add its monitors to the arrays
                if nodes[i][4]:
                    packetDelays.append(pDelay)
                    throughputs.append(thru)
        
            # If the device is a router ...
            elif nodes[i][3]:
                
                # Create the object, add it to the list of devices, and activate it
                router = Router(nodes[i][0],topology, self.globs)
                devices.append(router)
                activate(router, router.run())
            
            # Otherwise the input is incorrect ...
            else:
                print("Incorrect Input: One of the nodes is not a source, destination, or router.")
                assert(False)
                                    
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
    cmd = None
    while cmd != '':
        cmd = raw_input('Current parameters are:\n\n%s\n\n' % (sim.globs) +\
                        'Press Enter to start the simulation\n' +\
                        'or type <PARAMETER_NAME newValue> and press Enter '+\
                                                      'to change a parameter\n'+\
                        'or type ALL and press Enter to change all parameters\n')
        sim.globs.setParams(cmd)
    sim.run()
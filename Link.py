from SimPy.Simulation import *

class Link(Process):

    def __init__(self, linkRate, start, end, propTime):
        Process.__init__(self, name="Link"+str(linkID))
        self.queue = []
        self.linkRate = linkRate
        self.start = start
        self.end = end
        self.propTime = propTime
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
            # packet start propagate in the link
            packet.propTime = self.propTime
            packet.device = self.end
            reactivate(packet)
            print "t = %.2f: Job-%d departs" % (now(), packet.packetID)
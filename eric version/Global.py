class Global():

    def __init__(self):
        self.PACKET_SIZE = 8000

        self.INIT_WINDOW_SIZE = 1
        self.THRESHOLD = 1
        self.ACK_TIMEOUT = 1000 # NEED TO ESTIMATE LATER

        self.DYNAMIC_ROUTING = False
        self.PROBE_DROP_DELAY = 100
        self.PROBE_SAMPLE_SIZE = 50
        self.PROBE_RATE = 100

        self.DEFAULT_ALPHA = .6
        self.NUM_PACKETS_TO_TRACK_FOR_RTT = 10
        self.CONGESTION_CONTROL_ALGORITHM = "VEGAS"

        self.TEST_CASE = 2
        self.numFlows = 0
        self.flowsDone = 0
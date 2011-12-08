class Global():

    def __init__(self):
        self.PACKET_SIZE = 8000

        self.INIT_WINDOW_SIZE = 1
        self.THRESHOLD = 1
        self.ACK_TIMEOUT = 5000

        self.DYNAMIC_ROUTING = True
        self.PROBE_DROP_DELAY = 1000
        self.PROBE_SAMPLE_SIZE = 50
        self.PROBE_RATE = 100

        self.DEFAULT_ALPHA = .35
        self.NUM_PACKETS_TO_TRACK_FOR_RTT = 10
        self.CONGESTION_CONTROL_ALGORITHM = "VEGAS"

        self.TEST_CASE = 1
        self.THROUGHPUT_AVERAGE = 200
        
    def __str__(self):
        return('''\
    PACKET_SIZE = %s

    INIT_WINDOW_SIZE = %s
    THRESHOLD = %s
    ACK_TIMEOUT = %s

    DYNAMIC_ROUTING = %s
    PROBE_DROP_DELAY = %s
    PROBE_SAMPLE_SIZE = %s
    PROBE_RATE = %s

    DEFAULT_ALPHA = %s
    NUM_PACKETS_TO_TRACK_FOR_RTT = %s
    CONGESTION_CONTROL_ALGORITHM = %s

    TEST_CASE = %s
    THROUGHPUT_AVERAGE = %s'''\
        \
        % (self.PACKET_SIZE,\
           self.INIT_WINDOW_SIZE, self.THRESHOLD, self.ACK_TIMEOUT,\
           self.DYNAMIC_ROUTING, self.PROBE_DROP_DELAY, self.PROBE_SAMPLE_SIZE,\
               self.PROBE_RATE,\
           self.DEFAULT_ALPHA, self.NUM_PACKETS_TO_TRACK_FOR_RTT, self.CONGESTION_CONTROL_ALGORITHM,\
           self.TEST_CASE, self.THROUGHPUT_AVERAGE))
    
    def setParams(self, cmd):
        if cmd == 'ALL':
            for param in self.__dict__:
                value = raw_input('\nEnter new value for %s (enter nothing to leave value unchanged):\n' % (param))
                self.setParams('%s %s' % (param, value))
        cmd = cmd.split(None, 1)
        if len(cmd) == 2:
            (param, value) = cmd
            if param in self.__dict__:
                if param == 'DYNAMIC_ROUTING':
                    if value in ['True', 'False']:
                        self.__dict__[param] = eval(value)
                elif param == 'CONGESTION_CONTROL_ALGORITHM':
                    if value in ['VEGAS', 'AIMD', 'RENO']:
                        if value == 'RENO':
                            value = 'AIMD'
                        self.__dict__[param] = value
                elif value.isdigit():
                    if param != 'TEST_CASE' or value in ['1', '2']:
                        self.__dict__[param] = int(value)
                elif param in ['DEFAULT_ALPHA', 'PROBE_RATE', 'PROBE_DROP_DELAY', 'ACK_TIMEOUT']:
                    isFloat = True
                    for part in ('0'+value).split('.', 1):
                        if not part.isdigit():
                            isFloat = False
                    if isFloat:
                        self.__dict__[param] = float(value)
    
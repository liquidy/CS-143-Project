----CS 143 Project----

December 8, 2011

Authors:
 Mikhail Sushkov
 Eric Gomez
 Ben Weitz
 Judy Mou
 Kevin Lo

====================================================================================================================

The simulator can be configured during runtime by following the instructions
presented on your terminal.  The default parameter values can only be changed
by editing Global.py.  The configuration parameters are:

    PACKET_SIZE: packet size in bits
                 (integer)

    INIT_WINDOW_SIZE: initial source window size in packets
                      (integer)

    THRESHOLD: initial slow start threshold in packets
               (integer)

    ACK_TIMEOUT: acknowledgement timeout in milliseconds
                 (float or integer)

    DYNAMIC_ROUTING: whether or not to use dynamic routing
                     (boolean)

    PROBE_DROP_DELAY: packet delay in milliseconds recorded for a link when a
                      router probe is dropped
                      (float or integer)

    PROBE_SAMPLE_SIZE: number of delay data points to collect for a single link
                       before rerouting
                       (integer)

    PROBE_RATE: time in milliseconds to wait between sending router probes
                (float or integer)

    DEFAULT_ALPHA: the value of the parameter alpha used in TCP VEGAS; in units
                   of packets/millisecond
                   (float or integer)

    NUM_PACKETS_TO_TRACK_FOR_RTT: number of packets whose RTT is averaged to
                                  determined how to adjust the window size in TCP VEGAS
                                  (integer)

    AVERAGE_INTERVAL: The number of packets whose receiving times are averaged over to
                      determine the throughput and packeet delay. 
                      (integer)

    CONGESTION_CONTROL_ALGORITHM: "AIMD" or "VEGAS"

    TEST_CASE: 1 or 2 for the required test cases, or 'CUSTOM' to use the default values for
               NODES and TOPOLOGY defined in Global.py


*** NODES and TOPOLOGY can not be changed from the terminal; to change them, edit Global.py ***

    NODES: This is an array holding the data for the flows and routers. Each elemet of NODES is an array containing
           the data, [ID, isSource, isDestination, isRouter, isMonitored, sourceID, destID, bitsToSend, startTime].
           If the device is a source, it contains all these fields. If it is a Destination, it contains only the 
           fields [ID, isSource, isDestination, isRouter, isMonitored, sourceID, destID], and if it is a Router,
           it contains only [ID, isSource, isDestination, isRouter].
    
    TOPOLOGY: This is a matrix that stores the data for the links. If there are n nodes, then topology will be an 
              n x n matrix. The (i,j)th entry of TOPOLOGY is [-1] if there is no link between devices with ids 
              i and j, and [isMonitored, rate, propogationTime bufferCapacity] if there is a link between devices 
              i and j.

====================================================================================================================

To run the simulation, run Simulation.py and follow the instructions presented on your terminal.

====================================================================================================================

Graphs are produced in the working directory for:
    
    - buffer occupancies and dropped packets at monitored links
    - send rates and window sizes at each source
    - throughputs and packet delays at each destination

Each graph is saved as a .png image with a name indicating which test case was used, whether
dynamic routing was used, which congestion control algorithm was used, and what is shown on
the graph.

---- END README----
----CS 143 Project----

December 8, 2011

Authors:
 Mikhail Sushkov
 Eric Gomez
 Ben Weitz
 Judy Mou
 Kevin Lo

=============================================================================================

Configure the simulator by editing the default values for the fields in Global:

    PACKET_SIZE: packet size in bits

    INIT_WINDOW_SIZE: initial source window size in packets

    THRESHOLD: initial slow start threshold in packets

    ACK_TIMEOUT: acknowledgement timeout in milliseconds

    DYNAMIC_ROUTING: whether or not to use dynamic routing

    PROBE_DROP_DELAY: packet delay in milliseconds recorded for a link when a
                      router probe is dropped

    PROBE_SAMPLE_SIZE: number of delay data points to collect for a single link
                       before rerouting

    PROBE_RATE: time in milliseconds to wait between sending router probes

    DEFAULT_ALPHA: the value of the parameter alpha used in TCP VEGAS; in units
                   of packets/millisecond

    NUM_PACKETS_TO_TRACK_FOR_RTT: number of packets whose RTT is averaged to
                                  determined how to adjust the window size in TCP VEGAS
    
    THROUGHPUT_AVERAGE: 

    CONGESTION_CONTROL_ALGORITHM: "AIMD" or "VEGAS"

    TEST_CASE: 1 or 2 for the required test cases

________________________________________________________________________________
____TODO: explain input format__________________________________________________
________________________________________________________________________________

=============================================================================================

To run the simulation, simply run Simulation.py

=============================================================================================

Graphs are produced in the working directory for:
    
    - buffer occupancies and dropped packets at monitored links
    - send rates and window sizes at each source
    - throughputs and packet delays at each destination

Each graph is saved as a .png image with a name indicating which test case was used, whether
dynamic routing was used, which congestion control algorithm was used, and what is shown on
the graph.

---- END README----
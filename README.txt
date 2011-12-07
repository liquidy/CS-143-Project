----CS 143 Project----

December 8, 2011

Authors:
 Mikhail Sushkov
 Eric Gomez
 Ben Weitz
 Judy Mou
 Kevin Lo

=============================================================================================

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
    THROUGHPUT_AVERAGE: 

    CONGESTION_CONTROL_ALGORITHM: "AIMD" or "VEGAS"

    TEST_CASE: 1 or 2 for the required test cases

________________________________________________________________________________
____TODO: explain input format__________________________________________________
________________________________________________________________________________

=============================================================================================

To run the simulation, run Simulation.py and follow the instructions presented on your terminal.

=============================================================================================

Graphs are produced in the working directory for:
    
    - buffer occupancies and dropped packets at monitored links
    - send rates and window sizes at each source
    - throughputs and packet delays at each destination

Each graph is saved as a .png image with a name indicating which test case was used, whether
dynamic routing was used, which congestion control algorithm was used, and what is shown on
the graph.

---- END README----
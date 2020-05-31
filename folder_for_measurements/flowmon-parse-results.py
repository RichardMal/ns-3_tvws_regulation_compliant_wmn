# Author Richard Maliwatu
# Created: 10 April 2017
# Description: this file parses flowmonitor output in xml formart. It's based on
# original flowmon-parse-results.py at 
# https://www.nsnam.org/docs/release/3.15/doxygen/flowmon-parse-results_8py_source.html
#
# USAGE:
# python ./flowmon-parse-results.py <inputFile.xml> <outputfile.csv>
#
# Changes:
# 1. sends output to csv file
# 2. computes throughput
# 3. output file specified at runtime

# EDITING HINT:
# To get value from xml file:
# 1. ~ line 65, add element to "class Flow (object)" by including it in
# "_slots_= []" list and assigning value in "def_init_(self, flow-el):"
# 2. ~ line 177, add value to tuple in  "for sim in sim_list:"
# this might be a raw value or some calculated from the elements in the xml file

# FURTHER resources:
# Parsing xml file is not necessary [1]:
# 1. https://groups.google.com/forum/#!topic/ns-3-users/teE-cTfkV_Q
#
# some formulas[2]
# 2. http://mighthelpyou.blogspot.co.za/2016/04/flow-monitor-in-ns3-perfect-tool-for.html
#
# TO-DO
# i. Be sure about mean delay units, ms or s? 
# See line 199 "*1e3" and how that might be a conversion to ms. Be consistent with units
# ii. time ought to be in seconds for throughput computations, double check that necessary conversion is being done
# iii. pass outfile as runtime argument
from __future__ import division
import sys
import os
import csv

try:
    from xml.etree import cElementTree as ElementTree
except ImportError:
    from xml.etree import ElementTree

def parse_time_ns(tm):
    if tm.endswith('ns'):
        return long(tm[:-4])
    raise ValueError(tm)



class FiveTuple(object):
    __slots__ = ['sourceAddress', 'destinationAddress', 'protocol', 'sourcePort', 'destinationPort']
    def __init__(self, el):
        self.sourceAddress = el.get('sourceAddress')
        self.destinationAddress = el.get('destinationAddress')
        self.sourcePort = int(el.get('sourcePort'))
        self.destinationPort = int(el.get('destinationPort'))
        self.protocol = int(el.get('protocol'))
        
class Histogram(object):
    __slots__ = 'bins', 'nbins', 'number_of_flows'
    def __init__(self, el=None):
        self.bins = []
        if el is not None:
            #self.nbins = int(el.get('nBins'))
            for bin in el.findall('bin'):
                self.bins.append( (float(bin.get("start")), float(bin.get("width")), int(bin.get("count"))) )

class Flow(object):
    __slots__ = ['flowId', 'delayMean', 'packetLossRatio', 'rxBitrate', 'txBitrate',
                 'fiveTuple', 'packetSizeMean', 'probe_stats_unsorted',
                 'hopCount', 'flowInterruptionsHistogram', 'rx_duration', 'rxBytes', 
                 'timeLastRxPacket', 'timeFirstTxPacket', 'meanJitter']
    def __init__(self, flow_el):
        self.flowId = int(flow_el.get('flowId'))
        rxPackets = long(flow_el.get('rxPackets'))
        txPackets = long(flow_el.get('txPackets'))
        tx_duration = float(long(flow_el.get('timeLastTxPacket')[:-4]) - long(flow_el.get('timeFirstTxPacket')[:-4]))*1e-9
        rx_duration = float(long(flow_el.get('timeLastRxPacket')[:-4]) - long(flow_el.get('timeFirstRxPacket')[:-4]))*1e-9
        #get values necessary to compute throughput
        self.rxBytes = long(flow_el.get('rxBytes'))
        self.timeLastRxPacket = long(flow_el.get('timeLastRxPacket')[:-4]) #nano seconds
        self.timeFirstTxPacket = long(flow_el.get('timeFirstTxPacket')[:-4]) #nano seconds
        
        self.rx_duration = rx_duration
        
        #get jitter and guard against division by zero
        if ((rxPackets -1) > 0):
            self.meanJitter = float(flow_el.get('jitterSum')[:-4]) / (rxPackets-1) * 1e-9
        else:
            self.meanJitter = 0
        
        self.probe_stats_unsorted = []
        if rxPackets:
            self.hopCount = float(flow_el.get('timesForwarded')) / rxPackets + 1
        else:
            self.hopCount = -1000
        #delayMean is the average delay for the packets received
        if rxPackets:
            self.delayMean = float(flow_el.get('delaySum')[:-4]) / rxPackets * 1e-9
            self.packetSizeMean = float(flow_el.get('rxBytes')) / rxPackets           
        else:
            self.delayMean = None
            self.packetSizeMean = None
        if rx_duration > 0:
            self.rxBitrate = long(flow_el.get('rxBytes'))*8 / rx_duration
        else:
            self.rxBitrate = None
        if tx_duration > 0:
            self.txBitrate = long(flow_el.get('txBytes'))*8 / tx_duration
        else:
            self.txBitrate = None
        lost = float(flow_el.get('lostPackets'))
        #print "rxBytes: %s; txPackets: %s; rxPackets: %s; lostPackets: %s" % (flow_el.get('rxBytes'), txPackets, rxPackets, lost)
        if rxPackets == 0:
            self.packetLossRatio = None
        else:
            self.packetLossRatio = (lost / (rxPackets + lost))

        interrupt_hist_elem = flow_el.find("flowInterruptionsHistogram")
        if interrupt_hist_elem is None:
            self.flowInterruptionsHistogram = None
        else:
            self.flowInterruptionsHistogram = Histogram(interrupt_hist_elem)


class ProbeFlowStats(object):
    __slots__ = ['probeId', 'packets', 'bytes', 'delayFromFirstProbe']

class Simulation(object):
    def __init__(self, simulation_el):
        self.flows = []
        FlowClassifier_el, = simulation_el.findall("Ipv4FlowClassifier")
        flow_map = {}
        for flow_el in simulation_el.findall("FlowStats/Flow"):
            flow = Flow(flow_el)
            flow_map[flow.flowId] = flow
            self.flows.append(flow)
        for flow_cls in FlowClassifier_el.findall("Flow"):
            flowId = int(flow_cls.get('flowId'))
            flow_map[flowId].fiveTuple = FiveTuple(flow_cls)

        for probe_elem in simulation_el.findall("FlowProbes/FlowProbe"):
            probeId = int(probe_elem.get('index'))
            for stats in probe_elem.findall("FlowStats"):
                flowId = int(stats.get('flowId'))
                s = ProbeFlowStats()
                s.packets = int(stats.get('packets'))
                s.bytes = long(stats.get('bytes'))
                s.probeId = probeId
                if s.packets > 0:
                    s.delayFromFirstProbe =  parse_time_ns(stats.get('delayFromFirstProbeSum')) / float(s.packets)
                else:
                    s.delayFromFirstProbe = 0
                flow_map[flowId].probe_stats_unsorted.append(s)


def main(argv):
    file_obj = open(argv[1])
    print "Reading XML file ",
 
    sys.stdout.flush()        
    level = 0
    sim_list = []
    for event, elem in ElementTree.iterparse(file_obj, events=("start", "end")):
        if event == "start":
            level += 1
        if event == "end":
            level -= 1
            if level == 0 and elem.tag == 'FlowMonitor':
                sim = Simulation(elem)
                sim_list.append(sim)
                elem.clear() # won't need this any more
                sys.stdout.write(".")
                sys.stdout.flush()
    print " done."


    for sim in sim_list:
        #write output to csv file
        #*********************
        #uncomment the following line to hard code the output file
        #*********************
        #with open('/home/richard/eclipse_workspace/ns3_programming/folder_for_measurements/mesh_b.csv', "w") as out_file:
        #*****************************
        # A slight improvement, take second runtime argument as output file
        #*******************************************************************
        with open(argv[2], "w") as out_file: #take second runtime argument as output file
            writer = csv.writer(out_file, delimiter = ',', lineterminator='\n')
            header = ["FlowID", "TX bitrate (kbit/s)", "RX bitrate (kbit/s)", "meanDelay [s]",  "PacketLoss (%)", "Throughput (kbps)", "Mean jitter (s)", "Protocol"]
            writer.writerow(header)
            for flow in sim.flows:
                t = flow.fiveTuple
                proto = {6: 'TCP', 17: 'UDP'} [t.protocol]
                print "FlowID: %i (%s %s/%s --> %s/%i)" % \
                    (flow.flowId, proto, t.sourceAddress, t.sourcePort, t.destinationAddress, t.destinationPort)
                if flow.txBitrate is None:
                    print "\tTX bitrate: None"
                    txbitrate = 0
                else:
                    print "\tTX bitrate: %.2f kbit/s" % (flow.txBitrate*1e-3,)
                    txbitrate = flow.txBitrate*1e-3
                if flow.rxBitrate is None:
                    print "\tRX bitrate: None"
                    rxbitrate = 0
                else:
                    print "\tRX bitrate: %.2f kbit/s" % (flow.rxBitrate*1e-3,)
                    rxbitrate = flow.rxBitrate*1e-3
                if flow.delayMean is None:
                    print "\tMean Delay: None"
                    meanDelay = 0
                else:
                    print "\tMean Delay: %.2f ms" % (flow.delayMean*1e3,) #in ms
                    meanDelay =  flow.delayMean #this is in seconds
                if flow.packetLossRatio is None:
                    print "\tPacket Loss Ratio: None"
                    packetLoss = 0
                else:
                    print "\tPacket Loss Ratio: %.2f %%" % (flow.packetLossRatio*100)
                    packetLoss = flow.packetLossRatio*100
                # the "%" is only needed to control formating
                # output time in both nano seconds and seconds  just to be sure throughput calculation is happening correctly
                print " rxBytes: %.0f " % flow.rxBytes # "%.0f" implies zero decimal places
                print "timeLastRxPacket: %.0f" % flow.timeLastRxPacket, " (nano seconds) or ", float(flow.timeLastRxPacket)*1e-9, "(seconds)"
                print "timeFirstTxPacket: %.0f" % flow.timeFirstTxPacket, " (nano seconds) or ", float(flow.timeFirstTxPacket)*1e-9, "(seconds)"
                
                if flow.timeLastRxPacket == 0: #avoiud division by zero
                    throughput = 0
                else:
                    #throughput =flow.rxBytes * 8.0/(float(flow.timeLastRxPacket - flow.timeFirstTxPacket)*1e-9)/1024/1024 # Mbps
                    throughput =flow.rxBytes * 8.0/(float(flow.timeLastRxPacket - flow.timeFirstTxPacket)*1e-9)/1024 # kbps
                    #throughput =flow.rxBytes * 8.0/(float(flow.timeLastRxPacket - flow.timeFirstTxPacket)*1e-9)/1024 # bps
                
                print "Throughput: %.2f" % throughput, " kbps"
                #NTWENU = flow.meanNtwenu*1e3
                print " Mean jitter: %.2f " % flow.meanJitter
                jitter = flow.meanJitter
                
                row = [flow.flowId, txbitrate, rxbitrate, meanDelay, packetLoss,throughput, jitter, proto]
                writer.writerow(row)


if __name__ == '__main__':
    main(sys.argv)

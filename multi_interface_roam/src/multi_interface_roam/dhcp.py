#! /usr/bin/env python

import roslib; roslib.load_manifest('multi_interface_roam')
import asmach as smach
import scapy.all as scapy
from twisted.internet.defer import inlineCallbacks, returnValue
from netlink_monitor import monitor, IFSTATE
import async_helpers
import l2socket

class DhcpData:
    def __init__(self, iface):
        self.iface = iface
        self.socket = None

    def start_socket(self):
        self.socket = async_helpers.ReadDescrEventStream(l2socket.L2Port, iface = self.iface, filter='udp and dst port 68')

    def stop_socket(self):
        self.socket = None

    def send_discover(self):
        hwbytes = scapy.mac2str(self.hwaddr)
        pkt = (
              scapy.Ether(src=self.hwaddr, dst='ff:ff:ff:ff:ff:ff')/
              scapy.IP(src='0.0.0.0', dst='255.255.255.255')/
              scapy.UDP(sport=68, dport=67)/
              scapy.BOOTP(chaddr=[hwbytes])/
              scapy.DHCP(options=[
                  ("message-type", "discover"),
                  "end",
                  ])
              )
        self.socket.port.send(str(pkt))

class DhcpState(smach.State):
    def __init__(self, *args, **kwargs):
        smach.State.__init__(self, input_keys=['dhcp'], output_keys=['dhcp'], *args, **kwargs)


class Init(DhcpState):
    def __init__(self):
        DhcpState.__init__(self, outcomes=['sent'])

    @inlineCallbacks
    def execute_async(self, ud):
        yield async_helpers.async_sleep(1)
        ud.dhcp.start_socket()
        ud.dhcp.hwaddr = monitor.get_state_publisher(ud.dhcp.iface, IFSTATE.LINK_ADDR).get()
        ud.dhcp.send_discover()
        returnValue('sent')

class Receiving(DhcpState):
    def __init__(self):
        DhcpState.__init__(self, outcomes=['received'])
 
    @inlineCallbacks
    def execute_async(self, ud):
        print "Receiving"
        yield async_helpers.select(ud.dhcp.socket)
        pkt = ud.dhcp.socket.recv()
        print scapy.Ether()
        ud.dhcp.stop_socket()
        returnValue('received')

def start_dhcp(iface):
    sm = smach.StateMachine(outcomes=[], input_keys=['dhcp'])
    with sm:
        smach.StateMachine.add('INIT', Init(), transitions = {'sent':'RECEIVING'})
        smach.StateMachine.add('RECEIVING', Receiving(), transitions = {'received':'INIT'})

    ud = smach.UserData()
    ud.dhcp = DhcpData(iface)
    sm.execute_async(ud).addCallback(reactor.fireSystemEvent, 'shutdown')

if __name__ == "__main__":
    import sys
    from twisted.internet import reactor

    if len(sys.argv) != 2:
        print "usage: dhcp.py <interface>"
        sys.exit(1)

    iface = sys.argv[1]

    start_dhcp(iface)
    reactor.run()    

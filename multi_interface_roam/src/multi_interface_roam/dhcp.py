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
        self.first_time = True

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
        if ud.dhcp.first_time:
            ud.dhcp.first_time = False
            yield async_helpers.async_sleep(1)
        ud.dhcp.start_socket()
        ud.dhcp.hwaddr = monitor.get_state_publisher(ud.dhcp.iface, IFSTATE.LINK_ADDR).get()
        ud.dhcp.send_discover()
        returnValue('sent')

class Receiving(DhcpState):
    def __init__(self):
        DhcpState.__init__(self, outcomes=['received', 'exit'])
 
    @inlineCallbacks
    def execute_async(self, ud):
        print "Receiving"
        selectout = yield async_helpers.select(ud.dhcp.socket)
        #pkt = ud.dhcp.socket.recv()
        selectout = yield async_helpers.select(ud.dhcp.socket)
        pkt = ud.dhcp.socket.recv()
        #print repr(scapy.Ether(pkt))
        import weakref, gc
        wr = weakref.ref(ud.dhcp.socket)
        ud.dhcp.stop_socket()
        #async_helpers.follow_back(wr(), 5)
        import gcdebug
        #yield async_helpers.async_sleep(0.1)
        #sys.exc_clear()
        #gcdebug.dump_back_reference_graph(wr, 40)
        #returnValue('exit')
        returnValue('received')

def start_dhcp(iface):
    sm = smach.StateMachine(outcomes=['exit'], input_keys=['dhcp'])
    with sm:
        smach.StateMachine.add('INIT', Init(), transitions = {'sent':'RECEIVING'})
        smach.StateMachine.add('RECEIVING', Receiving(), transitions = {'received':'INIT'})

    ud = smach.UserData()
    ud.dhcp = DhcpData(iface)
    def shutdown(value):
        reactor.fireSystemEvent('shutdown')
    def ignore_eintr(error):
        if error.type == IOError:
            import errno
            if error.value.errno == errno.EINTR:
                return None
        return error
    sm.execute_async(ud).addCallback(shutdown).addErrback(ignore_eintr)

if __name__ == "__main__":
    import sys
    from twisted.internet import reactor

    if len(sys.argv) != 2:
        print "usage: dhcp.py <interface>"
        sys.exit(1)

    iface = sys.argv[1]

    start_dhcp(iface)
    reactor.run()    

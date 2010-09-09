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
        self.link_addr_state = monitor.get_state_publisher(self.iface, IFSTATE.LINK_ADDR) 

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

class Linkless(DhcpState):
    def __init__(self):
        DhcpState.__init__(self, outcomes=['got_replies', 'no_replies'])
 
    @inlineCallbacks
    def execute_async(self, ud):
        pass



class InitReboot(DhcpState):
    def __init__(self):
        DhcpState.__init__(self, outcomes=['got_replies', 'no_replies'])
 
    @inlineCallbacks
    def execute_async(self, ud):
        pass



class Rebooting(DhcpState):
    def __init__(self):
        DhcpState.__init__(self, outcomes=['got_replies', 'no_replies'])
 
    @inlineCallbacks
    def execute_async(self, ud):
        pass



class Init(DhcpState):
    def __init__(self):
        DhcpState.__init__(self, outcomes=['done'])

    @inlineCallbacks 
    def execute_async(self, ud):
        ud.dhcp.hwaddr = yield async_helpers.wait_for_state(ud.dhcp.link_addr_state, False, True)
        returnValue('done')



class Selecting(DhcpState):
    def __init__(self):
        DhcpState.__init__(self, outcomes=['got_replies', 'no_replies'])
 
    @inlineCallbacks
    def execute_async(self, ud):
        pass



class Requesting(DhcpState):
    def __init__(self):
        DhcpState.__init__(self, outcomes=['got_replies', 'no_replies'])
 
    @inlineCallbacks
    def execute_async(self, ud):
        ud.dhcp.start_socket()
        ud.dhcp.send_discover()
        selectout = yield async_helpers.select(ud.dhcp.socket)
        pkt = ud.dhcp.socket.recv()
        ud.dhcp.stop_socket()
        returnValue('got_replies')



class Bound(DhcpState):
    def __init__(self):
        DhcpState.__init__(self, outcomes=['got_replies', 'no_replies'])
 
    @inlineCallbacks
    def execute_async(self, ud):
        pass



class Renewing(DhcpState):
    def __init__(self):
        DhcpState.__init__(self, outcomes=['got_replies', 'no_replies'])
 
    @inlineCallbacks
    def execute_async(self, ud):
        pass



class Rebinding(DhcpState):
    def __init__(self):
        DhcpState.__init__(self, outcomes=['got_replies', 'no_replies'])
 
    @inlineCallbacks
    def execute_async(self, ud):
        pass



class Error(DhcpState):
    def __init__(self):
        DhcpState.__init__(self, outcomes=['got_replies', 'no_replies'])
 
    @inlineCallbacks
    def execute_async(self, ud):
        pass



def start_dhcp(iface):
    sm = smach.StateMachine(outcomes=['exit'], input_keys=['dhcp'])
    with sm:
        smach.StateMachine.add('INIT', Init(), transitions = {'done':'REQUESTING'})
        smach.StateMachine.add('REQUESTING', Requesting(), transitions = {'got_replies':'INIT', 'no_replies':'INIT'})

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

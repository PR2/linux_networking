#! /usr/bin/env python

import roslib; roslib.load_manifest('multi_interface_roam')

import socket
from scapy.all import BOOTP, DHCP, IP, UDP, Ether, mac2str
import IN
from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor, udp
from twisted.internet.defer import Deferred, DeferredQueue
import twisted.internet.error
import asmach as smach
import asmach.async as async
from netlink_monitor import monitor, IFSTATE
import sys
import pcap_twisted
from async_helpers import async_sleep, event_queue

#class Port(udp.Port):
#    """A special UDP port that binds to an interface by name and does
#    broadcast."""
#    def __init__(self, port = 68, *args, **kwargs):
#        udp.Port.__init__(self, port, *args, **kwargs)
#
#    def createInternetSocket(self):
#        s = udp.Port.createInternetSocket(self)
#        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, True)
#        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, True)
#        return s
#
#    def _bindSocket(self):
#        interface = self.interface
#        self.interface = ''
#        udp.Port._bindSocket(self)
#        self.interface = interface
#        self.socket.setsockopt(socket.SOL_SOCKET, IN.SO_BINDTODEVICE, self.interface+'\0')
#
#    def write(self, datagram, addr=None):
#        if addr is None:
#            if self.port == 67: addr = ('255.255.255.255', 68) 
#            else: addr = ('255.255.255.255', 67)
#        udp.Port.write(self, datagram, addr)
#
#def listenDhcp(protocol, interface, port = 68, *args, **kwargs):
#    p = Port(port, protocol, interface, *args, **kwargs)
#    p.startListening()
#    return p
#
#class MyProtocol(DatagramProtocol):
#    def datagramReceived(self, datagram, address):
#        print >> sys.stderr, "Got data!"

def send_discover2(s, hw):
    hwbytes = mac2str(hw)
    pkt = (
          Ether(src=hw, dst='ff:ff:ff:ff:ff:ff')/
          IP(src='0.0.0.0', dst='255.255.255.255')/
          UDP(sport=68, dport=67)/
          BOOTP(chaddr=[hwbytes])/
          DHCP(options=[
              ("message-type", "discover"),
              "end",
              ])
          )
    s.send(str(pkt))


class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['sent'],
            input_keys=['iface'],
            output_keys=['receiver', 'sender']
            )

    @async.function
    def execute_async(self, ud):
        yield
        print >> sys.stderr, "INIT"
        
        sender = pcap_twisted.RawSocket(ud.iface)
        ud.receiver = event_queue(pcap_twisted.Capture(ud.iface, 'udp and dst port 68'))
        
        hwaddr = monitor.get_state_publisher(ud.iface, IFSTATE.LINK_ADDR).get()
        send_discover2(sender, hwaddr)
        
        ud.sender = sender
        async.returnValue('sent')

class Receiving(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['received'],
            input_keys=['receiver', 'sender'],
            output_keys=['receiver', 'sender']
            )
 
    @async.function
    def execute_async(self, ud):
        #yield (async_sleep(2))
        print "Receiving"
        d = ud.receiver.get()
        yield d
        ud.receiver.unsubscribe(blocking = False)
        ud.sender.close()
        async.returnValue('received')

def start_dhcp(iface):
    sm = smach.StateMachine(outcomes=[], input_keys=['iface'])

    with sm:
        smach.StateMachine.add('INIT', Init(), transitions = {'sent':'RECEIVING'})
        smach.StateMachine.add('RECEIVING', Receiving(), transitions = {'received':'INIT'})

    ud = smach.UserData()
    ud.iface = iface
    sm.execute_async(ud)

if __name__ == "__main__":
    try:
        iface = 'eth0'
        #reactor.addSystemEventTrigger('during', 'shutdown', monitor.shutdown)
        reactor.addSystemEventTrigger('during', 'shutdown', sys.stdout.write, "shutting down")
        
        if False:
            import trace
            tracer = trace.Trace(ignoredirs = [sys.prefix, sys.exec_prefix])
            tracer.run("start_dhcp('eth0')")
        else:
            pass
            start_dhcp(iface) 
        print "Starting reactor"
        reactor.run()
    finally:
        monitor.shutdown()

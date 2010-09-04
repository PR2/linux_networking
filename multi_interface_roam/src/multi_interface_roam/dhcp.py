#! /usr/bin/env python

import roslib; roslib.load_manifest('multi_interface_roam')

import socket
from scapy.all import BOOTP, DHCP, IP, UDP, Ether, mac2str
import IN
from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor, udp
import twisted.internet.error
import asmach as smach
import asmach.async as async
from netlink_monitor import monitor, IFSTATE

class Port(udp.Port):
    """A special UDP port that binds to an interface by name and does
    broadcast."""
    def __init__(self, port = 68, *args, **kwargs):
        udp.Port.__init__(self, port, *args, **kwargs)

    def createInternetSocket(self):
        s = udp.Port.createInternetSocket(self)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, True)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, True)
        s.setsockopt(socket.SOL_SOCKET, IN.SO_BINDTODEVICE, self.interface+'\0')
        return s

    def _bindSocket(self):
        interface = self.interface
        self.interface = ''
        udp.Port._bindSocket(self)
        self.interface = interface

    def write(self, datagram, addr=None):
        if addr is None:
            if self.port == 67: addr = ('255.255.255.255', 68) 
            else: addr = ('255.255.255.255', 67)
        udp.Port.write(self, datagram, addr)

def listenDhcp(protocol, interface, port = 68, *args, **kwargs):
    p = Port(port, protocol, interface, *args, **kwargs)
    p.startListening()
    return p

class MyProtocol(DatagramProtocol):
    def datagramReceived(self, datagram, address):
        print "Got data!"

iface = 'eth0'
p = listenDhcp(MyProtocol(), iface)
hwbytes = mac2str(monitor.get_state_publisher(iface, IFSTATE.LINK_ADDR).get())
p.write(str(
          BOOTP(chaddr=[hwbytes])/
          DHCP(options=[
              ("message-type", "discover"),
              "end",
              ])
          ))

def send_discover(iface, hwaddr):
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

def send_discover2(s, hwaddr):
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
    s.write(str(pkt))

def send_discover3(iface, hwaddr):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, True)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, True)
    s.bind(("0.0.0.0", 68))
    s.setsockopt(socket.SOL_SOCKET, IN.SO_BINDTODEVICE, iface+'\0')

    hwbytes = mac2str(hw)
    pkt = (
          BOOTP(chaddr=[hwbytes])/
          DHCP(options=[
              ("message-type", "discover"),
              "end",
              ])
          )
    s.sendto(str(pkt), ('255.255.255.255', 67))
    print repr(BOOTP(s.recv(1500)))

#class NextState:
#    def __init__(self, state):
#        self.state = state
#
#class EndState:
#    def __init__(self, state):
#        self.state = state
#
#class CallLater:
#    def __init__(self, delay, cb):
#        self.id = reactor.callLater(delay, cb)
#
#    def __enter__(self):
#        pass
#
#    def __exit__(self, *args):
#        if self.id:
#            try:
#                self.id.cancel()
#            except twisted.internet.error.AlreadyCalled:
#                pass
#        else:
#            raise Exception("Foo Bar")
#        self.id = None
#
#class StateMachine:
#    def __init__(self, start_state):
#        self.enter_state(start_state)
#
#    def enter_state(self, state):
#        self.current_state = state(self.state_event)
#        self.current_state.next()
#
#    def state_event(self, *nargs, **kwargs):
#        try:
#            self.current_state.send((nargs, kwargs))
#        except NextState, ns:
#            self.enter_state(ns.state)
#        except EndState:
#            pass
#
#class TestStateMachine(StateMachine):
#    def __init__(self):
#        StateMachine.__init__(self, self.state1)
#
#    def state1(self, cb):
#        print "Entering state 1"
#        try:
#            with CallLater(0.1, cb):    
#                yield
#                return self.state2
#        finally:
#            print "Leaving state 1"
#
#    def state2(self, cb):
#        print "Entering state 2"
#        try:
#            with CallLater(0.1, cb):    
#                yield
#                return self.state1
#        finally:
#            print "Leaving state 2"
#
#class Dhcp:
#    def __init__(self, iface, hwaddr):
#        pcap_twisted.Capture(iface, 'udp and dst port 68').subscribe_repeating(self._recv_pkt)
#        self._raw_sock = pcap_twisted.RawSocket(iface)
#        self._hwaddr = hwaddr
#
#    def send_discover(self):
#        hwbytes = mac2str(hw)
#        pkt = (Ether(src=hw, dst='ff:ff:ff:ff:ff:ff')/
#              IP(src='0.0.0.0', dst='255.255.255.255')/
#              UDP(sport=68, dport=67)/
#              BOOTP(chaddr=[hwbytes])/
#              DHCP(options=[
#                  ("message-type", "discover"),
#                  "end",
#                  ])
#              )
#        self._raw_sock.write(str(pkt))
#
#    def send_dhcprequest(self):
#        pass
#
#    def state_machine(self):
#        while True:
#            if has_offer:
#                pass

class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                outcomes=['outcome1', 'outcome2'],
                output_keys=['dhcp_receiver']
                )

    @async.function
    def execute_async(self, ud):
        pass


if __name__ == "__main__":
    from twisted.internet import reactor
    from scapy.all import get_if_raw_hwaddr, mac2str
    import pcap_twisted
    from netlink_monitor import monitor, IFSTATE
    try:
        def got_pkt(len, data, time):
            print time, len, repr(Ether(data))

        print "GOOOOO!"

        #TestStateMachine()
        

        iface = 'eth0'
        hw = monitor.get_state_publisher(iface, IFSTATE.LINK_ADDR).get()
        for i in range(0,10):
            send_discover3(iface, hw)
        reactor.run()

    finally:
        monitor.shutdown()

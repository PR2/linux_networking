#! /usr/bin/env python

import socket
from scapy.all import BOOTP, DHCP, IP, UDP, Ether, mac2str
import IN
from twisted.internet import reactor
import twisted.internet.error
                  
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

class NextState:
    def __init__(self, state):
        self.state = state

class EndState:
    def __init__(self, state):
        self.state = state

class CallLater:
    def __init__(self, delay, cb):
        self.id = reactor.callLater(delay, cb)

    def __enter__(self):
        pass

    def __exit__(self, *args):
        if self.id:
            try:
                self.id.cancel()
            except twisted.internet.error.AlreadyCalled:
                pass
        else:
            raise Exception("Foo Bar")
        self.id = None

class StateMachine:
    def __init__(self, start_state):
        self.enter_state(start_state)

    def enter_state(self, state):
        self.current_state = state(self.state_event)
        self.current_state.next()

    def state_event(self, *nargs, **kwargs):
        try:
            self.current_state.send((nargs, kwargs))
        except NextState, ns:
            self.enter_state(ns.state)
        except EndState:
            pass

class TestStateMachine(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, self.state1)

    def state1(self, cb):
        print "Entering state 1"
        try:
            with CallLater(0.1, cb):    
                yield
                raise NextState(self.state2)
        finally:
            print "Leaving state 1"

    def state2(self, cb):
        print "Entering state 2"
        try:
            with CallLater(0.1, cb):    
                yield
                raise EndState(self.state1)
        finally:
            print "Leaving state 2"

class StateMachine2:
    def __init__(self, start_state):
        self.enter_state(start_state)

    def enter_state(self, state):
        self.current_state = state(self.state_event, self.enter_state)
        self.current_state.next()
        self.current_state.send(self.current_state.send)

class TestStateMachine2(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, self.state1)

    def state1(self, cb):
        myself = yield
        print "Entering state 1"
        try:
            with CallLater(0.1, myself):    
                yield
                raise NextState(self.state2)
        finally:
            print "Leaving state 1"

    def state2(self, cb):
        print "Entering state 2"
        try:
            with CallLater(0.1, myself):    
                yield
                raise EndState(self.state1)
        finally:
            print "Leaving state 2"

class Dhcp:
    def __init__(self, iface, hwaddr):
        pcap_twisted.Capture(iface, 'udp and dst port 68').subscribe_repeating(self._recv_pkt)
        self._raw_sock = pcap_twisted.RawSocket(iface)
        self._hwaddr = hwaddr

    def send_discover(self):
        hwbytes = mac2str(hw)
        pkt = (Ether(src=hw, dst='ff:ff:ff:ff:ff:ff')/
              IP(src='0.0.0.0', dst='255.255.255.255')/
              UDP(sport=68, dport=67)/
              BOOTP(chaddr=[hwbytes])/
              DHCP(options=[
                  ("message-type", "discover"),
                  "end",
                  ])
              )
        self._raw_sock.write(str(pkt))

    def send_dhcprequest(self):
        pass

    def state_machine(self):
        while True:
            if has_offer:
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

        TestStateMachine()
        

        #iface = 'eth0'
        #hw = monitor.get_state_publisher(iface, IFSTATE.LINK_ADDR).get()
        #for i in range(0,10):
        #    send_discover3(iface, hw)
        reactor.run()

    finally:
        monitor.shutdown()

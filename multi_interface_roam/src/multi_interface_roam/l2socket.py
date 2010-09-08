#! /usr/bin/env python

from __future__ import with_statement

from twisted.internet import reactor, interfaces
from zope.interface import implements
import scapy.all as scapy
import event
import socket

class AlreadyClosed(Exception):
    pass

class L2Port:
    implements(interfaces.IListeningPort)

    def __init__(self, proto, iface = 'any', filter = None, max_size = 9000, reactor = reactor):
        self._protocol = proto
        self._socket = scapy.L2Socket(iface, filter = filter).ins
        self._socket.setblocking(False)
        self._max_size = 9000
        self._protocol.makeConnection(self)
        self._reactor = reactor

    def fileno(self):
        if self._socket:
            return self._socket.fileno()
        else:
            return -1

    def doRead(self):
        data = self._socket.recv(self._max_size)
        self._protocol.dataReceived(data)

    def startListening(self):    
        self._reactor.addReader(self)

    def stopListening(self):
        self._reactor.removeReader(self)
        self.connectionLost()

    def send(self, data):
        self._socket.send(data)
    
    def connectionLost(self, reason=None):
        self._socket = None

    def logPrefix(self):
        return "L2Port"

if __name__ == "__main__":
    import unittest
    import async_helpers
    from async_helpers import unittest_with_reactor, async_test
    from twisted.internet.defer import Deferred
    from twisted.internet.protocol import Protocol
    import random
    
    def tst_icmp_pkt():
        return str(
                scapy.Ether()/
                scapy.IP(src='127.0.0.1', dst='127.0.0.1')/
                scapy.ICMP(type='echo-request', seq=1, id=random.randint(0, 0xFFFF))
                )

    class L2PortTest(unittest.TestCase):
        @async_test
        def test_basic(self):
            deferred = Deferred()
            packet = tst_icmp_pkt()
            class TstProto(Protocol):
                def dataReceived(self, data):
                    if data == packet:
                        deferred.callback(None)

            proto = TstProto()
            port = reactor.listenWith(L2Port, proto, iface = 'lo', filter='icmp')
            port.send(packet)
            yield deferred
             
        @async_test
        def test_as_event_stream(self):
            es = async_helpers.ReadDescrEventStream(L2Port, iface = 'lo', filter='icmp')
            pkt = tst_icmp_pkt()
            es.port.send(pkt)
            while True:
                yield async_helpers.select(es)
                inpkt = es.recv()
                if inpkt == pkt:
                    break

    def run_ros_tests():
        rostest.unitrun('multi_interface_roam', 'pcap_descriptor', L2PortTest)
    
    unittest_with_reactor(run_ros_tests)

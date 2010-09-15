#! /usr/bin/env python

import interface_upper
import ping_tester

class Interface:
    def __init__(self, iface):
        self.iface = iface
        ping_tester.PingTester(iface, 20, ('prbase7', 1194))

class DhcpInterface(Interface):
    def __init__(self, iface):
        Interface.__init__(self, iface)
        interface_upper.InterfaceUpper(iface)

class WirelessInterface(DhcpInterface):
    def __init__(self, iface):
        DhcpInterface.__init__(self, iface)

DhcpInterface('eth0')
import twisted.internet.reactor as reactor
reactor.run()

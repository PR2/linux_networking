#! /usr/bin/env python

import interface_upper
import ping_tester
import dhcp
from dhcp_apply_config import DhcpAddressSetter, DhcpRouteSetter

class Interface:
    def __init__(self, iface, tableid):
        self.iface = iface
        ping_tester.PingTester(iface, 20, ('prbase7', 1194))

class DhcpInterface(Interface):
    def __init__(self, iface, tableid):
        Interface.__init__(self, iface, tableid)
        interface_upper.InterfaceUpper(iface)
        dhcpdata = dhcp.dhcp_client(iface)
        DhcpAddressSetter(iface, dhcpdata.binding_publisher)
        DhcpRouteSetter(iface, tableid, dhcpdata.binding_publisher)
        
        # FIXME RPFilter somewhere.
        # Flush ip rule on interface up somewhere.
        # Set ip rule for ping somewhere.

class WirelessInterface(DhcpInterface):
    def __init__(self, iface, tableid):
        DhcpInterface.__init__(self, iface, tableid)



DhcpInterface('eth1', 50)
import twisted.internet.reactor as reactor
def preshutdown():
    print "Booh!"
reactor.addSystemEventTrigger('before', 'shutdown', preshutdown)
reactor.run()
print "Done"
import threading
import time
while True:    
    threads = threading.enumerate()
    non_daemon = sum(0 if t.daemon else 1 for t in threads)
    if non_daemon == 1:
        break
    print
    print "Remaining threads:", non_daemon, len(threads)
    for t in threads:
        print ("daemon: " if t.daemon else "regular:"), t.name
    time.sleep(1)

#! /usr/bin/env python

import interface_upper
import ping_tester
import dhcp
from dhcp_apply_config import DhcpAddressSetter, DhcpRouteSetter, DhcpSourceRuleSetter
import config

class Interface:
    def __init__(self, iface, tableid):
        self.iface = iface
        src_rule_setter = DhcpSourceRuleSetter(iface, tableid, tableid)
        self.ping_tester = ping_tester.PingTester(iface, 20, ('prbase7', 1194), src_rule_setter.state_pub)

    def periodic_update(self, interval):
        self.ping_tester.update()

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

class StaticRouteInterface(Interface):
    pass

class NoType(Exception):
    pass

def construct(iface, tableid):
    try:
        type = config.get_interface_parameter(iface, 'type')
        if type == "wired":
            return DhcpInterface(iface, tableid)
        if type == "wireless":
            return WirelessInterface(iface, tableid)
        if type == "static":
            return StaticRouteInterface(iface, tableid)
    except config.NoDefault:
        raise NoType()


if __name__ == "__main__":
    try:
        DhcpInterface('eth1', 50)
    except:
        import traceback
        traceback.print_exc()
    import twisted.internet.reactor as reactor
    reactor.run()
    import threading
    import time
    while True:    
        time.sleep(0.1)
        threads = threading.enumerate()
        non_daemon = sum(0 if t.daemon else 1 for t in threads)
        if non_daemon == 1:
            break
        print
        print "Remaining threads:", non_daemon, len(threads)
        for t in threads:
            print ("daemon: " if t.daemon else "regular:"), t.name

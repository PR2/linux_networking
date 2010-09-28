#! /usr/bin/env python

import roslib; roslib.load_manifest('multi_interface_roam')
from state_publisher import StatePublisher, CompositeStatePublisher
import network_monitor_udp.udpmoncli as udpmoncli
from netlink_monitor import IFSTATE, netlink_monitor
from twisted.internet import reactor

state_publishers = {}

def get_state_publisher(iface):
    if iface not in state_publishers:
        state_publishers[iface] = StatePublisher(False)
    return state_publishers[iface]
    
class PingTester:
    def __init__(self, iface, rate, pingtarget, state_pub):
        self.state_publisher = get_state_publisher(iface)
        reactor.addSystemEventTrigger('before', 'shutdown', self._shutdown)
        self.udp_monitor = udpmoncli.MonitorClient([.2], pingtarget, rate, 32, True)
        CompositeStatePublisher(lambda (addr, ready): None if not ready else addr, [
            netlink_monitor.get_state_publisher(iface, IFSTATE.ADDR),
            state_pub,
        ]).subscribe(self._addr_cb)

    def update(self, update_rate):
        self.state_publisher.set(self.udp_monitor.get_smart_bins(update_rate))
        return self.state_publisher.get()

    def _addr_cb(self, old_state, new_state):
        if new_state:
            # FIXME This should only happen after routes are set up.
            self.udp_monitor.start_monitor((new_state[0], 0))
            print "Starting monitor on %s"%new_state[0]
        else:
            self.udp_monitor.stop_monitor()
            self.state_publisher.set(None)

    def _shutdown(self):
        try:
            self.udp_monitor.shutdown()
        except:
            import sys
            sys.print_exc()

#! /usr/bin/env python

import system
from netlink_monitor import netlink_monitor, IFSTATE
from state_publisher import CompositeStatePublisher
from twisted.internet import reactor

# TODO Add tests.

class InterfaceUpper:
    def __init__(self, iface):
        self.iface = iface
        CompositeStatePublisher(lambda x: x, [
            netlink_monitor.get_state_publisher(iface, IFSTATE.PLUGGED),
            netlink_monitor.get_state_publisher(iface, IFSTATE.UP),
        ]).subscribe(self._cb)
        self._is_shutdown = False
        self.state = None
        reactor.addSystemEventTrigger('before', 'shutdown', self._shutdown)
   
    def restart():
        return system.system('ifconfig', self.iface, 'down')

    def _cb(self, old_state, new_state):
        plugged, up = new_state
        self.state = new_state
        if plugged and not up and not self._is_shutdown:
            system.system('ifconfig', self.iface, 'up')

    def _shutdown(self):
        self._is_shutdown = True
        if self.state:
            system.system('ifconfig', self.iface, 'down')

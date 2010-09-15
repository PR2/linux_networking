#! /usr/bin/env python

import system
from netlink_monitor import netlink_monitor, IFSTATE
from state_publisher import CompositeStatePublisher

# TODO Add tests.

class InterfaceUpper:
    def __init__(self, iface):
        self.iface = iface
        self.pub = CompositeStatePublisher(lambda x: x, [
            netlink_monitor.get_state_publisher(iface, IFSTATE.PLUGGED),
            netlink_monitor.get_state_publisher(iface, IFSTATE.UP),
        ]).subscribe(self._cb)
   
    def restart():
        return system.system('ifconfig', self.iface, 'down')

    def _cb(self, old_state, new_state):
        plugged, up = new_state
        if plugged and not up:
            system.system('ifconfig', self.iface, 'up')

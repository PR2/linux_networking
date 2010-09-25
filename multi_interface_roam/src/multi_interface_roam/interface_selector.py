#! /usr/bin/env python

import interface
import event
from twisted.internet import reactor

class RULEID:
    LOCAL=50
    FIRST_IFACE=100
    TUNNEL=150
    BLOCK_TUNNEL=175
    DEFAULT=200 
    BLOCK_NON_TUNNEL=250

class InterfaceSelector:
    def __init__(self):
        self.interfaces = {}
        self.update_event = event.Event()

        interface_names = config.get_parameter('interfaces').keys()

        ifaceid = RULEID.FIRST_IFACE
        for iface in interface_names:
            try:
                self.interfaces[iface] = interface.construct(iface, ifaceid)
                ifaceid += 1
            except interface.NoType:
                print << sys.stderr, "Interface %s has no type."%iface
                continue
            except:
                print << sys.stderr, "Error creating interface %s."%iface
                print_exc()

        reactor.callLater(1, self._periodic_update)

    def _periodic_update(self):
        for iface in self.interface.values:
            iface.update()
        self.update_event.trigger()

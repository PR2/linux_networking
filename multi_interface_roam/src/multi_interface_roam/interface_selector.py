#! /usr/bin/env python

import interface
import event
from twisted.internet import reactor
from twisted.internet.defer import inlineCallbacks
import ip_rule
import config
import time
import sys
import radio_manager

class RULEID:
    LOCAL=100
    FIRST_IFACE=50
    TUNNEL=150
    BLOCK_TUNNEL=175
    DEFAULT=200 
    BLOCK_NON_TUNNEL=250

class InterfaceSelector:
    def __init__(self):
        self.interfaces = {}
        self.update_event = event.Event()
        self.update_interval = 1
        self.radio_manager = radio_manager.RadioManager()
        self.inactive_penalty = config.get_parameter('inactive_penalty', 50)

        # Add rules to guarantee that local routes go to the main table.
        local_net_rule = ip_rule.IpRule(RULEID.LOCAL)
        for subnet in config.get_parameter('local_networks'):
            local_net_rule.add('to', subnet, 'lookup', 'main')

        # Create all the interfaces.
        interface_names = config.get_parameter('interfaces').keys()
        ifaceid = RULEID.FIRST_IFACE
        for iface in interface_names:
            try:
                new_iface = self.interfaces[iface] = interface.construct(iface, ifaceid)
                ifaceid += 1
            except interface.NoType:
                print >> sys.stderr, "Interface %s has no type."%iface
                sys.exit(1)
            except interface.UnknownType, e:
                print >> sys.stderr, "Interface %s has unknown type %s."%(iface, e)
                sys.exit(1)
            except:
                print >> sys.stderr, "Error creating interface %s."%iface
                raise

        # Register the radios with the radio manager
        for i in self.interfaces.itervalues():
            if isinstance(i, interface.WirelessInterface):
                self.radio_manager.add_iface(i)
        
        # Prepare the rules that select a particular interface.
        self.tun_ip_rules = [ip_rule.IpRule(RULEID.TUNNEL+i) for i in range(len(self.interfaces))]
        
        # Set up periodic updates, and run the first one.
        self.shutting_down = False
        self._periodic_update()
        reactor.addSystemEventTrigger('before', 'shutdown', self._shutdown)

    def _shutdown(self):
        self.shut_down = True

    def set_mode(self, ssid = "", bssid = "", sel_interface = "", use_tunnel = True, band = 3):
        self.radio_manager.set_mode(ssid, bssid, band)

    def _periodic_update(self):
        if self.shutting_down:
            return
        self.periodic_update_handle = reactor.callLater(self.update_interval, self._periodic_update)
        
        # Update all the interfaces.
        for iface in self.interfaces.values():
            iface.update(self.update_interval)
        
        # Update the radio manager
        self.radio_manager.update()

        # Rank the interfaces.
        self.rank_interfaces()

        # Broadcast the fact that an update has just completed.
        self.update_event.trigger()
    
    @inlineCallbacks
    def set_tun_rules(self, selected_interfaces):
        # Set the interfaces we are given in order.
        for i, iface in enumerate(selected_interfaces):
            self.tun_ip_rules[i].set('lookup', iface.tableid)

        # Clear the remaining rules.
        for tir in self.tun_ip_rules[len(selected_interfaces):]:
            yield tir.set()
    
    TERRIBLE_INTERFACE = -1e1000

    def score_interface(self, iface):
        # score is used for selecting the interface.
        # prescore is used by radio manager to decide which interface to
        # activate.
        
        if iface.goodness <= 0:
            iface.prescore = iface.score = InterfaceSelector.TERRIBLE_INTERFACE
            return

        iface.prescore = iface.score = iface.goodness + iface.reliability + iface.priority
        if not iface.active:
            iface.score -= self.inactive_penalty

    def rank_interfaces(self):        
        # Score interfaces
        interfaces = self.interfaces.values()
        for iface in interfaces:
            self.score_interface(iface)

        # Sort, and forget about the scores
        interfaces.sort(key = lambda iface: iface.score, reverse = True)
        active_interfaces = [ iface for iface in interfaces if iface.score != self.TERRIBLE_INTERFACE ]
        
        # Set the interfaces
        self.set_tun_rules(active_interfaces)

        # Print active_iface status
        now = time.time()
        print
        print time.ctime(now), now
        for rank, iface in enumerate(interfaces):
            # FIXME
            iface.timeout_time = now
            active = "active" if iface.active else ""
            print "#% 2i %10.10s %7.1f %7.3f %17.17s %7.3f %3.0f %s"% \
                    (rank, iface.name, (iface.timeout_time - now), iface.score, iface.bssid, iface.goodness, iface.reliability, active)

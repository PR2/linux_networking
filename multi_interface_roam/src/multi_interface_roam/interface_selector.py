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
import logging_config
import socket
import system
from netlink_monitor import netlink_monitor, IFSTATE
from async_helpers import mainThreadCallback

summary_logger = logging_config.get_logger_stream_for_file('console.summary')

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
        self.forced_interface = ""
        self.use_tunnel = True
        self.tunnel_interface = config.get_parameter('tunnel_interface', "")
        self.active_interfaces = []

        print "Resolving basestation IP. (Blocking operation.)"
        self.basestation_ip = socket.gethostbyname(config.get_parameter('base_station'))

        # Add rules to guarantee that local routes go to the main table.
        local_net_rule = ip_rule.IpRule(RULEID.LOCAL)
        for subnet in config.get_parameter('local_networks'):
            local_net_rule.add('to', subnet, 'lookup', 'main')

        # Add a rule to send through the vpn.
        if self.tunnel_interface:
            self.vpn_rule = ip_rule.IpRule(RULEID.DEFAULT)
            # Use LINK here because netlink_monitor's parsing rules don't
            # currently work on vpn interfaces.
            system.system('ip', 'route', 'flush', 'table', str(RULEID.DEFAULT))
            netlink_monitor.get_state_publisher(self.tunnel_interface,
                    IFSTATE.LINK).subscribe(self._refresh_default_route)


        # Create all the interfaces.
        interface_names = config.get_parameter('interfaces').keys()
        ifaceid = RULEID.FIRST_IFACE
        for iface in interface_names:
            try:
                new_iface = self.interfaces[iface] = interface.construct(iface, ifaceid)
                new_iface.score = InterfaceSelector.TERRIBLE_INTERFACE 
                new_iface.prescore = InterfaceSelector.TERRIBLE_INTERFACE 
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

    @mainThreadCallback
    def set_mode(self, ssid = "", bssid = "", sel_interface = "", use_tunnel = True, band = 3, scan_only = False):
        print >> sys.stderr, "Dynamic reconfiguration ssid: %s bssid: %s iface: %s tun: %s band: %s scan_only: %s"%(ssid, bssid, sel_interface, use_tunnel, band, scan_only)
        self.goodness_weight = config.get_parameter('ping_weighting', 0.5)
        self.radio_manager.set_mode(ssid, bssid, band, scan_only, sel_interface)
        self.forced_interface = sel_interface
        self.use_tunnel = use_tunnel
        if use_tunnel:
            self.vpn_rule.set('lookup', str(RULEID.DEFAULT))
        else:
            self.vpn_rule.set()

    def _refresh_default_route(self, old_state, new_state):
        if new_state:
            system.system('ip', 'route', 'replace', 'table', str(RULEID.DEFAULT), 'default', "dev", self.tunnel_interface)

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
            if self.use_tunnel:
                self.tun_ip_rules[i].set('to', self.basestation_ip, 'lookup', iface.tableid)
            else:
                self.tun_ip_rules[i].set('lookup', iface.tableid)

        # Clear the remaining rules.
        for tir in self.tun_ip_rules[len(selected_interfaces):]:
            yield tir.set()
    
    TERRIBLE_INTERFACE = -1e1000

    def score_interface(self, iface):
        # score is used for selecting the interface.
        # prescore is used by radio manager to decide which interface to
        # activate.
        
        if iface.goodness <= 0 and self.forced_interface != iface.iface:
            iface.prescore = iface.score = InterfaceSelector.TERRIBLE_INTERFACE
            return

        # If an interface is being forced, other interfaces
        # should all have terrible scores.
        if self.forced_interface and self.forced_interface != iface.iface:
            iface.prescore = iface.score = InterfaceSelector.TERRIBLE_INTERFACE
            return

        iface.prescore = iface.score = self.goodness_weight * iface.goodness + (1 - self.goodness_weight) * iface.reliability + iface.priority
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
        print >> summary_logger
        print >> summary_logger, time.ctime(now), now
        #print >> summary_logger, netlink_monitor.get_status_publisher(self.tunnel_interface).get()
        for rank, iface in enumerate(interfaces):
            # FIXME
            iface.timeout_time = now
            active = "active" if iface.active else ""
            rule = "rule  " if iface in active_interfaces else "norule"
            print >> summary_logger, "#% 2i %10.10s %7.1f %7.3f %7.3f %17.17s %7.3f %3.0f %s %s"% \
                    (rank, iface.prettyname, (iface.timeout_time - now), iface.score, iface.prescore, iface.bssid, iface.goodness, iface.reliability, rule, active)

        self.active_interfaces = active_interfaces

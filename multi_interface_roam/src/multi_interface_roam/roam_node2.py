#! /usr/bin/env python

import roslib; roslib.load_manifest('multi_interface_roam')
import rospy
import config
import dynamic_reconfigure.server
import twisted.internet.reactor as reactor
from multi_interface_roam.cfg import MultiInterfaceRoamConfig
import mac_addr
import interface_selector
import sys

# FIXME May want to kill this at some point
import asmach as smach
smach.logdebug = lambda x: None

class Node:
    def __init__(self, *args, **kwargs):
        rospy.init_node(*args, **kwargs)
        rospy.core.add_shutdown_hook(self._shutdown_by_ros)
        reactor.addSystemEventTrigger('after', 'shutdown', self._shutdown_by_reactor)

    def _shutdown_by_reactor(self):
        rospy.signal_shutdown("Reactor shutting down.")

    def _shutdown_by_ros(self, why):
        reactor.fireSystemEvent('shutdown')

class RoamNode:
    def __init__(self):
        Node("multi_interface_roam")
        self.interface_selector = interface_selector.InterfaceSelector()
        self.reconfig_server = dynamic_reconfigure.server.Server(MultiInterfaceRoamConfig, self.reconfigure)

    def reconfigure(self, config, level):
        if config['interface'] not in self.interface_selector.interfaces:
            config['interface'] = ''
        interface = config['interface']
        ssid = config['ssid'] = config['ssid'][0:32]
        bssid = ""
        if not mac_addr.is_str(config['bssid']):
            config['bssid'] = ""
        else:
            bssid = mac_addr.str_to_packed(config['bssid'])
        use_tunnel = config['use_tunnel']

        self.interface_selector.set_mode(ssid, bssid, interface, use_tunnel, config['band'])

        return config

if __name__ == "__main__":
    try:
        RoamNode()
    except:
        import traceback
        traceback.print_exc()
        print >> sys.stderr, "\nCaught exception during startup. Shutting down."
        reactor.fireSystemEvent('shutdown')
    reactor.run()

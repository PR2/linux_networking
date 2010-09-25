#! /usr/bin/env python

import roslib; roslib.load_manifest('multi_interface_roam')
import rospy
import config
import dynamic_reconfigure.server
from twisted.internet import reactor
from multi_interface_roam.cfg import MultiInterfaceRoamConfig
import mac_addr

class RoamNode:
    def __init__():
        rospy.init_node("multi_interface_roam")
        self.reconfig_server = dynamic_reconfigure.server.Server()

    def reconfigure(config, level):
        if config['interface'] not in self.interface_manager.interfaces():
            config['interfaces'] = ''
        ssid = config['ssid'] = config['ssid'][0:32]
        if not mac_addr.is_str(config['bssid']):
            config['bssid'] = ""
        bssid = config['bssid']
        use_tunnel = config['use_tunnel']

        self.interface_manager.set_mode(ssid, bssid, interface, use_tunnel)

        return config

if __name__ == "__main__":
    RoamNode()
    reactor.start()

#! /usr/bin/env python

import roslib; roslib.load_manifest('multi_interface_roam')
import rospy
import config
import dynamic_reconfigure.server
import twisted.internet.reactor as reactor
from multi_interface_roam.cfg import MultiInterfaceRoamConfig
from multi_interface_roam.msg import MultiInterfaceStatus, InterfaceStatus
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue, DiagnosticArray
from pr2_msgs.msg import AccessPoint
import mac_addr
import interface_selector
import sys
from std_msgs.msg import Int32
import sigblock
import signal
import interface
from ieee80211_channels.channels import IEEE80211_Channels

# FIXME May want to kill this at some point
import asmach as smach
smach.logdebug = lambda x: None

class Node:
    def __init__(self, *args, **kwargs):
        sigblock.save_mask()
        sigblock.block_signal(signal.SIGCHLD)
        rospy.init_node(*args, **kwargs)
        sigblock.restore_mask()
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
        self._interfaces = self.interface_selector.interfaces.values()
        
        # Prepare topics to publish
        pub_namespace = rospy.remap_name('wifi')
        self.diag_pub = rospy.Publisher("/diagnostics", DiagnosticArray)
        self.ap_pub = rospy.Publisher(pub_namespace+"/accesspoint", AccessPoint)
        self.status_pub = rospy.Publisher(pub_namespace+"/status", MultiInterfaceStatus)
        self.iface_id_pub = rospy.Publisher(pub_namespace+'/current_iface_id', Int32, latch = True)
        self._wireless_interfaces = [ i for i in self._interfaces if i.__class__ == interface.WirelessInterface ]
        self.all_ap_pub = dict((iface, rospy.Publisher(pub_namespace+"/"+iface.iface+"/accesspoint", AccessPoint)) for iface in self._wireless_interfaces)
        
        # Kick off publication updates.
        self.interface_selector.update_event.subscribe_repeating(self._publish_status)

    def _publish_status(self):
        now = rospy.get_rostime()

        # current_iface_id
        ai = self.interface_selector.active_interfaces
        if not ai or ai[0] not in self._interfaces:
            index = -1
        else:
            index = self._interfaces.index(ai[0])
        self.iface_id_pub.publish(index)

        # accesspoint
        best_active = self.interface_selector.radio_manager.best_active
        for iface in self._wireless_interfaces:
            msg = self.gen_accesspoint_msg(iface)
            msg.header.stamp = now
            self.all_ap_pub[iface].publish(msg)
            if iface == best_active:
                self.ap_pub.publish(msg)
        if best_active is None:
            self.ap_pub.publish(AccessPoint())

        # status
        msg = MultiInterfaceStatus()
        #msg.

        # diagnostics

    @staticmethod
    def gen_accesspoint_msg(iface):
        msg = AccessPoint()
        msg.essid = iface.essid
        msg.macaddr = iface.bssid
        msg.signal = iface.wifi_signal 
        msg.noise = iface.wifi_noise
        msg.snr = msg.signal - msg.noise
        msg.quality = iface.wifi_quality
        msg.rate = iface.wifi_rate
        msg.tx_power = iface.wifi_txpower
        msg.channel = IEEE80211_Channels.get_channel(iface.wifi_frequency * 1e6)
        return msg

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

        self.interface_selector.set_mode(ssid, bssid, interface, use_tunnel, config['band'], config['scan_only'])

        return config

if __name__ == "__main__":
    def start():
        try:
            RoamNode()
        except:
            import traceback
            traceback.print_exc()
            print >> sys.stderr, "\nCaught exception during startup. Shutting down."
            reactor.fireSystemEvent('shutdown')
    reactor.addSystemEventTrigger('before', 'startup', start)
    reactor.run()

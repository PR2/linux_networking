#! /usr/bin/env python

import socket
from scapy.layers.all import BOOTP, DHCP, IP, UDP
                  
def send_raw_packet(iface, data):
    s = socket.socket(socket.PF_PACKET, socket.SOCK_DGRAM)
    s.bind((iface,0x0800))
    s.send(data)

class Dhcp:
    def __init__(self):
        pass



if __name__ == "__main__":
    iface = 'lo'
    from scapy.all import get_if_raw_hwaddr
    from netlink_monitor import monitor
    hw = monitor.get_state_publisher(iface, )
    send_raw_packet(iface, str( 
            IP(src='0.0.0.0', dst='255.255.255.255')/
            UDP(sport=68, dport=67)/
            BOOTP(chaddr=hw)/
            DHCP(options=[
                ("message-type", "discover"),
                "end",
                ])
            ))

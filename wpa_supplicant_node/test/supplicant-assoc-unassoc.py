#! /usr/bin/env python

# Expects wpa_supplicant to be running as:
# wpa_supplicant -i wlan1 -c /u/blaise/temp/net.conf -C /var/run/wpa_supplicant

from __future__ import with_statement

import subprocess
import time
import sys

def call(cmd):
    print "\nCMD-->", "'"+("' '".join(cmd))+"'"
    with open('/dev/null', 'w') as dev_null:
        subprocess.call(cmd, stdout = dev_null)

def wpa_cli(iface, cmd):
    call(['wpa_cli', '-p', '/var/run/wpa_supplicant', '-i', iface] + cmd)

def disconnect(iface, network_id):
    if network_id is None:
        call(['iw', 'dev', iface, 'disconnect'])
    else:
        wpa_cli(iface, ['disconnect'])

def connect(iface, essid, freq, bssid, network_id):
    if network_id is None:
        call(['iw', 'dev', iface, 'connect', essid, freq, bssid])
    else:
        #wpa_cli(iface, ['select_network', network_id])
        #wpa_cli(iface, ['bssid', network_id, bssid])
        wpa_cli(iface, ['reconnect'])

def is_connected(iface):
    p = subprocess.Popen(['iw', 'dev', iface, 'link'], stdout = subprocess.PIPE)
    stdout, _ = p.communicate()
    if stdout.startswith("Not connected"):
        return False
    elif stdout.startswith("Connected to"):
        return True
    elif stdout.startswith("Authenticated"):
        return None
    else:
        raise Exception("Unexpected response:\n%s"%stdout)

def wait_connect_state(iface, state, timeout = 10):
    end_time = time.time() + timeout
    while time.time() < end_time:
        if is_connected(iface) == state:
            return True
        time.sleep(0.1)
    return False

def cycle_test(iface, essid, freq, bssid, network_id):
    cycles = 0
    successes = 0
    if network_id is None:
        call([ 'ifconfig', iface, 'up' ])
    while True:
        cycles += 1
        print "Unassociating...",
        sys.stdout.flush()
        disconnect(iface, network_id)
        if not wait_connect_state(iface, False):
            print "\nFailed to get interface in disconnected state. Bailing!"
            break;
        print "done.",
        sys.stdout.flush()
        time.sleep(0.5)
        
        print "Associating...",
        sys.stdout.flush()
        connect(iface, essid, freq, bssid, network_id)
        if not wait_connect_state(iface, True):
            print "FAIL.",
        else:
            print "done.",
            successes += 1
        print "Success %i/%i"%(successes, cycles)
        sys.stdout.flush()
        time.sleep(0.5)

# Parameters are all strings:
# interface, essid, frequency, bssid, wpa_supplicant network id. 
cycle_test('wlan2', 'blaise-test', '5240', '00:25:9C:C4:D7:7E', '0')

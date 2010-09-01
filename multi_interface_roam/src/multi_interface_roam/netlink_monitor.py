#! /usr/bin/env python

import command_with_output
import state_publisher
import threading
import time
import traceback


# FIXME Move this elsewhere.
import subprocess
class RunCommand:
    def __init__(self, *args):
        proc = subprocess.Popen(list(args), stdout = subprocess.PIPE, stderr = subprocess.PIPE, close_fds = True)
        (self.stdout, self.stderr) = proc.communicate()

class NetlinkMonitor(command_with_output.CommandWithOutput):
    IFSTATE_PLUGGED = 0
    IFSTATE_UP = 1
    IFSTATE_LINK = 2
    IFSTATE_ADDR = 3

    def __init__(self):
        self.lock = threading.RLock()
        self.raw_state_publishers = [{}, {}, {}, {}]
        self.state_publishers = [{}, {}, {}, {}]
        self.cur_iface = None
        self.deleted = None
        command_with_output.CommandWithOutput.__init__(self, ['ip', 'monitor', 'link', 'addr'], 'ip_monitor')

    def child_restart(self):
        time.sleep(0.2) # Limit race conditions on getting the startup state.
        current_state = RunCommand('ip', 'addr')
        with self.lock:
            old_cur_iface = self.cur_iface
            old_deleted = self.deleted
            for line in current_state.stdout.split('\n'):
                self.got_line(line)
            self.deleted = old_deleted
            self.cur_iface = old_cur_iface

    def get_state_publisher(self, interface, level):
        if level == 0:
            return self.get_raw_state_publisher(interface, level)
        pubs =  self.state_publishers[level]
        if not interface in pubs:
            pubs[interface] = state_publisher.CompositeStatePublisher(lambda l: l[0] and l[1], [
                    self.get_raw_state_publisher(interface, level - 1),
                    self.get_raw_state_publisher(interface, level), 
                    ])
        return pubs[interface]
    
    def get_raw_state_publisher(self, interface, level):
        pubs =  self.raw_state_publishers[level]
        if not interface in pubs:
            pubs[interface] = state_publisher.StatePublisher(False)
        return pubs[interface]
    
    def got_line(self, line):
        with self.lock:
            try:
                # Figure out the interface, and whether this is a delete
                # event.
                
                if len(line) == 0 or (line[0] == ' ' and self.cur_iface == None):
                    return
                tokens = line.rstrip().split()
                link_info = False
                if line[0] != ' ':
                    if tokens[0] == 'Deleted':
                        self.deleted = True
                        tokens.pop(0)
                    else:
                        self.deleted = False
                    self.cur_iface = tokens[1].rstrip(':')
                    if tokens[1][-1] == ':':
                        link_info = True
                    tokens.pop(0)
                    tokens.pop(0)

                if link_info:
                    # Plugged or not?
                    self.get_raw_state_publisher(self.cur_iface, self.IFSTATE_PLUGGED).set(not self.deleted)
                    
                    # Up or not?
                    flags = tokens[0].strip('<>').split(',')
                    self.get_raw_state_publisher(self.cur_iface, self.IFSTATE_UP).set('UP' in flags)

                    # Have a link?
                    state_idx = tokens.index('state')
                    state = tokens[state_idx + 1]
                    self.get_raw_state_publisher(self.cur_iface, self.IFSTATE_LINK).set(state != 'DOWN')
                
                else:
                    # Find the address.
                    try:
                        addr_idx = tokens.index('inet') 
                        if self.deleted:
                            addr_state = False
                        else:
                            addr_state = tokens[addr_idx + 1].split('/')
                        self.get_raw_state_publisher(self.cur_iface, self.IFSTATE_ADDR).set(addr_state)
                    except ValueError:
                        pass
            except Exception, e:
                print "Caught exception in NetlinkMonitor.run:", e
                traceback.print_exc(10)
                print

monitor = NetlinkMonitor()

if __name__ == "__main__":
    while True:
        for i in range(0,4):
            print monitor.get_raw_state_publisher('wlan2', i).get(),
            print monitor.get_state_publisher('wlan2', i).get(), '  /  ',
        print
        time.sleep(1)



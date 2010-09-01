#! /usr/bin/env python

from __future__ import with_statement

from twisted.internet import reactor
import pcap
import event
import threading

class AlreadyClosed(Exception):
    pass

class Capture(event.Event):
    class _CaptureDescriptor:
        def __init__(self, parent, iface = 'any', filter = None, snaplen = 65535, promisc = False, timeout = 10):
            self.parent = parent
            self.pcap = pcap.pcapObject()
            self.pcap.open_live(iface, snaplen, promisc, 0)
            if filter is not None:
                self.pcap.setfilter(filter, 1, 0)
            self.pcap.setnonblock(True)
            print "starting"
            reactor.addReader(self)
    
        def fileno(self):
            if self.pcap is None:
                return -1
            return self.pcap.fileno()
    
        def connectionLost(self, reason):
            if self.parent:
                self.parent.trigger(None, None, None)
    
        def logPrefix(self):
            return ""
    
        def doRead(self):
            if self.parent:
                self.pcap.dispatch(-1, self.parent.trigger)
        
        def close(self):
            print "stopping"
            reactor.removeReader(self)
            self.pcap = None
            self.parent = None

    def __init__(self, *args, **kwargs):
        self._descr = None
        self._args = args
        self._kwargs = kwargs
        self._sub_change_lock = threading.Lock()
        event.Event.__init__(self)

    def _subscription_change(self):
        with self._sub_change_lock:
            has_descr = self._descr is not None
            has_subs = len(self._subscribers) != 0
            if has_descr and not has_subs:
                self._descr.close()
                self._descr = None
            elif not has_descr and has_subs:
                if self._args != None: # We are not closed.
                    self._descr = Capture._CaptureDescriptor(self, *self._args, **self._kwargs)
                else:
                    # There is a race that lets a subscribe slip through
                    # just after close does its unsubscribe_all. This case patches it.
                    self.unsubscribe_all()

    def subscribe(self, *args, **kwargs):
        if self._args is None:
            raise AlreadyClosed("Attempted to subscribe to a closed Capture object.")
        return event.Event.subscribe(self, *args, **kwargs)

    def close(self):
        with self._sub_change_lock:
            self._args = None
            self._kwargs = None
        self.unsubscribe_all()

    def __enter__(self):
        pass

    def __exit__(self, *args):
        self.close()

if __name__ == "__main__":
    def print_pkt(iface, len, data, time):
        print time, iface, len, repr(data)

    import gc
    def hasPcapObj():
        count = 0
        for obj in gc.get_objects():
            #t = type(obj)
            if isinstance(obj, pcap.pcapObject) or \
                    isinstance(obj, event.EventCallbackHandle) or \
                    isinstance(obj, Capture._CaptureDescriptor) or \
                    isinstance(obj, Capture):
                if isinstance(obj, Capture):
                    print obj, gc.get_referents(gc.get_referents(obj))
                count += 1
        print "objects found:", count
   
    class DeathReporter:
        def __init__(self, name):
            self.name = name
        
        def __del__(self, *args):
            print "I'm dead!", self.name

    hasPcapObj()
    c1 = Capture('lo', 'icmp')
    #c1.foo = DeathReporter('c1')
    #c2 = Capture('eth0', 'icmp')
    h = c1.subscribe_repeating(print_pkt, 'lo')
    #c2.subscribe_repeating(print_pkt, 'eth0')
    #reactor.callLater(0.1, c1.close)
    reactor.callLater(1, hasPcapObj)
    reactor.callLater(2, h.unsubscribe)
    reactor.callLater(3, hasPcapObj)
    reactor.callLater(4, c1.subscribe_repeating, print_pkt, 'lo')
    reactor.callLater(5, hasPcapObj)
    reactor.callLater(6, c1.close)
    reactor.callLater(7, hasPcapObj)
    reactor.callLater(8, c1.subscribe_repeating, print_pkt, 'lo')
    reactor.callLater(9, hasPcapObj)
    print type(h)
    print type(c1)
    print c1.__class__, Capture
    del c1
    del h
    reactor.run()
    hasPcapObj()

    

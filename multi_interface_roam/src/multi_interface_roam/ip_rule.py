#! /usr/bin/env python

from twisted.internet.defer import inlineCallbacks, DeferredLock
from twisted.internet import reactor
import state_publisher
import system

class IpRule:
    def __init__(self, priority):
        self._lock = DeferredLock()
        self.priority = str(priority)
        self.is_shutdown = False
        self.state_pub = state_publisher.StatePublisher(())
        self._flush()
        reactor.addSystemEventTrigger('before', 'shutdown', self._shutdown)

    @inlineCallbacks
    def _flush(self):
        self.prev = () # Won't hurt to do it here, just means we might not
                       # delete during ongoing set. Ensures it is set at
                       # end of construction.
        yield self._lock.acquire()
        try:
            retcode = None
            while not retcode:
                retcode = yield system.system('ip', 'rule', 'del', 'priority', self.priority)
        finally:
            self._lock.release()
    
    @inlineCallbacks
    def set(self, *args):
        """Changes/adds the rule. Call with no arguments to clear the rule."""
        if self.is_shutdown:
            return
        yield self._lock.acquire()
        try:
            # Add new rule
            if args:
                yield system.system('ip', 'rule', 'add', "priority", self.priority, *args)
            # Add remove old rule
            if self.prev:
                yield system.system('ip', 'rule', 'del', "priority", self.priority, *self.prev)
            self.prev = args
            self.state_pub.set(args)

        finally:
            self._lock.release()

    @inlineCallbacks
    def _shutdown(self):
        yield self.set()
        self.is_shutdown = True

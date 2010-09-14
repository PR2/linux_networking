#! /usr/bin/env python

import logging
import logging.handlers
import os
from logging_config import *
from twisted.internet import protocol, reactor
from twisted.internet.defer import Deferred, inlineCallbacks
import sys

class System(protocol.ProcessProtocol):
    def __init__(self, *args):
        self.deferred = Deferred()
        self.proc = None
        self.shutdown_trigger = reactor.addSystemEventTrigger('during', 'shutdown', self.shutdown)
        self.proc = reactor.spawnProcess(self, args[0], args, None)
    
    def errReceived(self, data):
        print >> sys.stderr, data

    def outReceived(self, data):
        print >> sys.stdout, data

    def processEnded(self, status_object):
        reactor.removeSystemEventTrigger(self.shutdown_trigger)
        self.deferred.callback(status_object.value.exitCode)

    def shutdown(self):
        if self.proc:
            self.proc.signalProcess("INT")

def system(*args):
    s = System(*args)
    return s.deferred

if __name__ == "__main__":
    import unittest
    from async_helpers import unittest_with_reactor, async_test
    import time

    class SystemTest(unittest.TestCase):
        @async_test
        def test_basic(self):
            """Runs a simple command."""
            retval = yield system('echo', 'Hello')
            self.assertEqual(retval, 0)

        @async_test
        def test_retval(self):
            """Checks that return values work."""
            retval = yield system('sh', '-c', 'exit 42')
            self.assertEqual(retval, 42)

        @async_test
        def test_no_return_early(self):
            """Checks that System waits until termination."""
            start = time.time()
            retval = yield system('sleep', '1')
            duration = time.time() - start
            self.assertEqual(retval, 0)
            self.assertTrue(duration >= 1, "%s > 1 is false"%duration)

    def run_ros_tests():
        rostest.unitrun('multi_interface_roam', 'system', SystemTest)
    
    unittest_with_reactor(run_ros_tests)

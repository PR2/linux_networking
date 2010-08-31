#! /usr/bin/env python

from __future__ import with_statement

import event

class StatePublisher():
    def __init__(self, init_state):
        self._set_lock = event.ReentrantDetectingLock()
        self._state = init_state
        self._event = event.Event()

    def get(self):
        return self._state

    def set(self, new_state):
        with self._set_lock("Tried to set state from state callback."):
            old_state = self._state
            self._state = new_state
            self._event.trigger(old_state = old_state, new_state = new_state)
            self._setting_thread = None

    def subscribe(self, args, kwargs):
        with self._set_lock("Tried to subscribe from state callback."):
            h = self._event.subscribe_repeating(args, kwargs)
            h._trigger((), {'old_state' : None, 'new_state' : self._state})
        return h

if __name__ == "__main__":
    import roslib; roslib.load_manifest('multi_interface_roam')
    import rostest
    import unittest
    import sys
    
    def state_logger(l, old_state, new_state):
        l.append((old_state, new_state))

    class BasicTest(unittest.TestCase):
        def test_basic(self):
            s = StatePublisher(False)
            l = []
            h1 = s.subscribe(state_logger, l)
            h2 = s.subscribe(state_logger, l)
            s.set(1)
            h1.unsubscribe()
            s.set(2)
            h2.unsubscribe()
            self.assertEqual(l, [
                (None, False),
                (None, False),
                (False, 1),
                (False, 1),
                (1, 2),
                ])

    if len(sys.argv) > 1 and sys.argv[1].startswith("--gtest_output="):
        rostest.unitrun('multi_interface_roam', 'state_publisher_basic', BasicTest)
    else:
        unittest.main()

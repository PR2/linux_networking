#! /usr/bin/env python

from __future__ import with_statement

import threading
import weakref

class DeadlockException(Exception):
    pass

class MultipleUnsubscribe(Exception):
    pass

class EventCallbackHandle:
    def __init__(self, event, cb, args, nargs, repeating):
        # Use a weakref to avoid creating a loop for the GC.
        self._event = weakref.ref(event)
        self._cb = cb
        self._args = args
        self._nargs = nargs
        self._call_lock = threading.Lock()
        self._repeating = repeating
        self._running_thread = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.unsubscribe()

    def _trigger(self, args, nargs):
        # Prepare the positional parameters
        allargs = self._args + args
    
        # Prepare the named parameters. Parameters given to
        # the subscribe call have precedence.
        allnargs = dict(nargs)
        allnargs.update(self._nargs)
    
        # Call the callback
        cb = self._cb
        ct = threading.current_thread()
        if self._running_thread == ct:
            raise DeadlockException("Callback recursively triggered itself.")
        with self._call_lock:
            if cb is not None:
                if not self._repeating:
                    self.unsubscribe(blocking = False, _clear_repeating = False)
                self._running_thread = ct
                cb(*allargs, **allnargs)
                self._running_thread = None
    
    def unsubscribe(self, blocking = True, _clear_repeating = True):
        # Kill as many references as we can.
        # Clear _cb first as it is the one that gets checked in _trigger. 
        self._cb = None
        # If not an automatic unsubscribe then set _repeating to true so
        # that the next unsubscribe can raise a MultipleUnsubscribe
        # exception. 
        if _clear_repeating:
            self._repeating = True
        self._args = ()
        self._nargs = ()
        event = self._event()
        if event is not None:
            try:
                del event._subscribers[self]
            except KeyError:
                # Don't check once callbacks because there is a race
                # between manual deletion and automatic deletion. If a once
                # callback is manually cleared, _repeating will be true, so
                # we will catch the second attempt.
                if self._repeating:
                    raise MultipleUnsubscribe("Repeating callback unsubscribed multiple times.")
                else:
                    self._repeating = True
        if blocking:
            if self._running_thread == threading.current_thread():
                raise DeadlockException("Callback tried to blocking unsubscribe itself.")
            with self._call_lock:
                pass

class Event:
    def __init__(self, name = "Unnamed Event"): 
        self._name = name
        self._subscribers = {}

    def subscribe(self, cb, args, nargs, repeating = True):
        """Subscribes to an event. 
        
        Can be called at any time and from any thread. Subscriptions that
        occur while an event is being triggered will not be called until
        the next time the event is triggered."""

        h = EventCallbackHandle(self, cb, args, nargs, repeating)
        self._subscribers[h] = None
        return h
    
    def subscribe_once(*args, **nargs):
        # We don't want the names we use to limit what the user can put in
        # nargs. So do all our arguments positional.
        return args[0].subscribe(args[1], args[2:], nargs, repeating = False)

    def subscribe_repeating(*args, **nargs):
        # We don't want the names we use to limit what the user can put in
        # nargs. So do all our arguments positional.
        return args[0].subscribe(args[1], args[2:], nargs, repeating = True)

    def trigger(*args, **nargs):
        """Triggers an event.

        Concurrent triggers of a given callback are serialized using a lock, so 
        triggering from a callback will cause a deadlock."""
        
        self = args[0]
        args = args[1:]

        for h in self._subscribers.keys():
            h._trigger(args, nargs)

if __name__ == "__main__":
    import roslib; roslib.load_manifest('multi_interface_roam')
    import rostest
    import unittest
    import sys
        
    def append_cb(l, *args, **nargs):
        l.append((args, nargs))

    class BasicTest(unittest.TestCase):
        def test_basic(self):
            """Tests basic functionality.
            
            Adds a couple of callbacks. Makes sure they are called the
            right number of times. Checks that parameters are correct,
            including keyword arguments giving priority to subscribe over
            trigger."""
            e = Event()
            l1 = []
            l2 = []
            h1 = e.subscribe_repeating(append_cb, l1, 'd', e = 'f')
            e.subscribe_once(append_cb, l2, 'a', b = 'c')
            e.trigger('1', g = 'h')
            e.trigger('2', e = 'x')
            h1.unsubscribe()
            e.trigger('3')
            sys.stdout.flush()
            self.assertEqual(l1, [
                (('d', '1'), { 'e' : 'f', 'g' : 'h'}), 
                (('d', '2'), { 'e' : 'f'}),
                ])
            self.assertEqual(l2, [
                (('a', '1'), { 'b' : 'c', 'g': 'h'}),
                ])

        def test_unsub_myself(self):
            """Tests that a callback can unsubscribe itself."""
            e = Event()
            l = []
            class Unsubscriber:
                def cb(self, *args, **nargs):
                    append_cb(l, *args, **nargs)
                    self.h.unsubscribe(blocking = False)
            u = Unsubscriber()
            u.h = e.subscribe_repeating(u.cb)
            e.trigger('t1')
            e.trigger('t2')
            self.assertEqual(l, [
                (('t1',), {}),
                ])

        def test_unsub_myself_blocking(self):
            """Tests that a blocking unsubscribe on myself raises exception."""
            e = Event()
            l = []
            class Unsubscriber:
                def cb(self, *args, **nargs):
                    append_cb(l, *args, **nargs)
                    self.h.unsubscribe(blocking = True)
            u = Unsubscriber()
            u.h = e.subscribe_repeating(u.cb)
            self.assertRaises(DeadlockException, e.trigger, ['t1'])

    def wait_cv(cv, l, cb, trig):
        with cv:
            l.append((cb, trig, 'pre'))
            cv.notify_all()
            cv.wait()
            l.append((cb, trig, 'post'))

    class ThreadTest(unittest.TestCase):
        def setUp(self):
            self.e = Event()
            self.cv = threading.Condition()
            self.l = []
            self.h1 = self.e.subscribe_once(wait_cv, self.cv, self.l, 'cb1')
            self.h2 = self.e.subscribe_once(wait_cv, self.cv, self.l, 'cb1')
            self.t = threading.Thread(target = self.e.trigger, args=['t1'])
            self.t.start()

        def test_norun_sub_during_trig(self):
            """Tests that a callback that gets added during a trigger is
            not run."""
            
            # Trigger event
            with self.cv:
                # Handle first callback.
                while not self.l:
                    self.cv.wait()
                # This runs while wait_cv is waiting
                self.l.append('main')
                self.e.subscribe_repeating(append_cb, self.l, 'cb2')
                self.cv.notify_all()

                # Handle second wait_cv callback.
                while len(self.l) != 4:
                    self.cv.wait()
                self.l.append('main2')
                self.cv.notify_all()
            
            # Let the trigger finish
            self.t.join()

            self.expected = [
                ('cb1', 't1', 'pre'), 
                'main',
                ('cb1', 't1', 'post'), 
                ('cb1', 't1', 'pre'), 
                'main2',
                ('cb1', 't1', 'post'), 
                (('cb2', 't2'), {}),
                ]
            
            # Trigger event again
            self.e.trigger('t2')
            
            self.assertEqual(self.l, self.expected)

        def test_norun_unsub_during_trig(self):
            """Tests that a callback that gets deleted during a trigger is
            not run."""
            
            # Trigger event
            with self.cv:
                # Handle first callback.
                while not self.l:
                    self.cv.wait()
                # This runs while wait_cv is waiting
                self.l.append('main')
                unsubed = 0
                try:
                    self.h1.unsubscribe(blocking = False)
                    unsubed += 1
                except MultipleUnsubscribe:
                    pass
                try:
                    self.h2.unsubscribe(blocking = False)
                    unsubed += 1
                except MultipleUnsubscribe:
                    pass
                self.assertEqual(unsubed, 1)
                self.cv.notify_all()
            
            # Let the trigger finish
            self.t.join()

            self.expected = [
                ('cb1', 't1', 'pre'), 
                'main',
                ('cb1', 't1', 'post'), 
                ]

            # Trigger event again
            self.e.trigger('t2')
            
            self.assertEqual(self.l, self.expected)

    if False:
        rostest.unitrun('multi_interface_roam', 'event_basic', BasicTest)
        rostest.unitrun('multi_interface_roam', 'event_thread', ThreadTest)
    else:
        unittest.main()

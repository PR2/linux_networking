#! /usr/bin/env python

from __future__ import with_statement

import threading

# TODO:
# - Safely unsubscribing from a nonrepeating event. Currently you might
# delete just after a trigger deleted, and a subscribe added with the same
# id.

class DeadlockException(Exception):
    pass

class EventCallbackHandle:
    def __init__(self, id, event):
        self.id = id
        self._event = event

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self._event._unsubscribe(id)

    def unsubscribe(self, blocking = True):
        self._event._unsubscribe(self.id, blocking)

class Event:
    def __init__(self, name = "Unnamed Event"): 
        self._name = name
        self._subscribers = {} 
        self._next_id = 0 
        self._subscribe_lock = threading.Lock() 
        self._unsubscribe_cv = threading.Condition()
        self._trigger_lock = threading.Lock()
        self._running_callback = None
        self._unsubscribe_list = None
        self._trigger_thread = None

    def subscribe(self, cb, args, nargs, repeating = True):
        """Subscribes to an event. 
        
        Can be called at any time and from any thread. Subscriptions that
        occur while an event is being triggered will not be called until
        the next time the event is triggered."""

        with self._subscribe_lock:
            # Allocate an id
            # Fails if each int has been used.
            # Robust to next_id wrapping around.
            while self._next_id in self._subscribers:
                #or \
                #  self._next_id in self._unsubscribe_list: 
                self._next_id += 1
            id = self._next_id
            self._next_id += 1

            self._subscribers[id] = (cb, args, nargs, repeating)
    
            return EventCallbackHandle(id, self)
    
    def subscribe_once(*args, **nargs):
        # We don't want the names we use to limit what the user can put in
        # nargs. So do all our arguments positional.
        return args[0].subscribe(args[1], args[2:], nargs, repeating = False)

    def subscribe_repeating(*args, **nargs):
        # We don't want the names we use to limit what the user can put in
        # nargs. So do all our arguments positional.
        return args[0].subscribe(args[1], args[2:], nargs, repeating = True)

    def _unsubscribe(self, id, blocking = True):
        with self._unsubscribe_cv:
            while self._running_cb == id and blocking:
                if self._trigger_thread == threading.current_thread():
                    raise DeadlockException("Event callback doing blocking unsubscribe of itself.")
                self._unsubscribe_cv.wait()
            del self._subscribers[id]
            if self._unsubscribe_list is not None:
                self._unsubscribe_list.append(id)

    def trigger(*args, **nargs):
        """Triggers an event.

        Concurrent triggers are serialized using a lock, so triggering from
        a callback will cause a deadlock."""
        
        self = args[0]
        args = args[1:]

        # Clear _unsubscribe_list, as any items currently on it
        with self._trigger_lock:
            self._trigger_thread = threading.current_thread()
            with self._unsubscribe_cv:
                self._unsubscribe_list = []
                items = self._subscribers.items()
            
            for (id, (cb, cbargs, cbnargs, repeating)) in items:
                with self._unsubscribe_cv:
                    # Make a note of the currently running callback
                    self._running_cb = id
                    
                    # Notify waiting unsubscribers that we have changed 
                    # callback.
                    self._unsubscribe_cv.notify_all()

                    # Delete the current callback if it doesn't repeat.
                    if not repeating and id not in self._unsubscribe_list:
                        del self._subscribers[id]

                # Call the callback unless it has been unsubscribed while
                # we were triggering.
                if id not in self._unsubscribe_list:
                    # Prepare the positional parameters
                    allargs = cbargs + args

                    # Prepare the named parameters. Parameters given to
                    # the subscribe call have precedence.
                    allnargs = dict(nargs)
                    allnargs.update(cbnargs)

                    # Call the callback
                    cb(*allargs, **allnargs)
            
            with self._unsubscribe_cv:
                # Make a note that no callback is currently running
                self._running_cb = None

                # Clear the list of callbacks deleted while we were
                # triggering.
                self._unsubscribe_list = None
            self._trigger_thread = None

if __name__ == "__main__":
    import roslib; roslib.load_manifest('multi_interface_roam')
    import rostest
    import unittest
    import sys
        
    def append_cb(l, *args, **nargs):
        l.append((args, nargs))

    def wait_cv(cv, l, cb, trig):
        with cv:
            l.append((cb, trig, 'pre'))
            cv.notify_all()
            cv.wait()
            l.append((cb, trig, 'post'))

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

    class ThreadTest(unittest.TestCase):
        def setUp(self):
            self.e = Event()
            self.cv = threading.Condition()
            self.l = []
            self.h1 = self.e.subscribe_once(wait_cv, self.cv, self.l, 'cb1')
            self.h2 = self.e.subscribe_once(wait_cv, self.cv, self.l, 'cb1')
            threading.Thread(target = self.e.trigger, args=['t1']).start()

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

            # Trigger event again
            self.e.trigger('t2')

            self.assertEqual(self.l, [
                ('cb1', 't1', 'pre'), 
                'main',
                ('cb1', 't1', 'post'), 
                ('cb1', 't1', 'pre'), 
                'main2',
                ('cb1', 't1', 'post'), 
                (('cb2', 't2'), {}),
                ])

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
                except KeyError:
                    pass
                try:
                    self.h2.unsubscribe(blocking = False)
                    unsubed += 1
                except KeyError:
                    pass
                self.assertEqual(unsubed, 1)
                self.cv.notify_all()

            # Trigger event again
            self.e.trigger('t2')

            self.assertEqual(self.l, [
                ('cb1', 't1', 'pre'), 
                'main',
                ('cb1', 't1', 'post'), 
                ])

    if False:
        rostest.unitrun('multi_interface_roam', 'event_basic', BasicTest)
        rostest.unitrun('multi_interface_roam', 'event_thread', ThreadTest)
    else:
        unittest.main()

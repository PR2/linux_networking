from twisted.internet.defer import Deferred, DeferredQueue
from twisted.internet import reactor

def async_sleep(t):
    d = Deferred()
    reactor.callLater(t, d.callback, None)
    return d

def event_queue(event):
    q = DeferredQueue()
    def cb(*args, **kwargs):
        q.put((args, kwargs))
    h = event.subscribe_repeating(cb)
    q.unsubscribe = h.unsubscribe
    return q


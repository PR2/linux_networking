#! /usr/bin/env python
            
import traceback
import sys

class Return:
    def __init__(self, value = None):
        self.value = value
        raise self


def function(func):
    def function_internal(*args, **kwargs):
        f = func(*args, **kwargs)
        entry = yield
        f.next()
        m = f.send
        args = (None, )
        while True:
            try:
                yield_val = m(*args)
            except StopIteration:
                raise Return(None)
            else:
                try:
                    m = f.send
                    if yield_val is None:
                        # We are yielding all the way back.
                        args = yield
                        args = (args, )
                    else:
                        # f is calling an asynchronous function.
                        m = f.send
                        r = yield_val
                        r.next()
                        val = (entry, )
                        while True:
                            try:
                                r.send(val)
                                val = yield
                            except Return, r:
                                args = (r.value, )
                                break
                except:
                    m = f.throw
                    args = list(sys.exc_info())
                    # Remove misleading lines from the trace.
                    try:
                        args[2] = args[2].tb_next.tb_next
                    except:
                        pass
    
    return function_internal

def start(f):
    r = f
    r.next()
    r.send(r)
    return r

@function
def print_result(f, name = "<unnamed>"):
    yield
    try:
        out = yield f
        print name, "returned:", out
    except:
        print name, "had exception:"
        tb = list(sys.exc_info())
        #tb[2] = tb[2].tb_next.tb_next
        traceback.print_exception(*tb)

def run(f, name = "<unnamed>"):
    try:
        r = start(print_result(f, name))
        while True:
            r.send(None)
    except Return, r:
        pass

#from twisted.internet import defer
#
#def start(f):
#    d = defer.Deferred()
#    f.next()
#    d.addCallbacks(f.send, f.throw)
#    return d
#
#def run(f, name = "<unnamed>"):
#    try:
#        r = start(print_result(f, name))
#        while True:
#            print "iter"
#            r = r.callback(None)
#    except Return, r:
#        pass
#
#def Return(value = None):
#    returnValue(value)

@function
def f(a, b):
    yield
    Return(a + b)

@function
def g2():
    yield
    raise Exception("Booh!")

@function
def g(a, b):
    yield
    yield
    yield g2()
    Return(5)

@function
def h(a, b):
    yield
    out = yield (g(a, b))
    Return(out)

@function
def i(a, b, c):
    yield
    out = yield (f(a, b))
    out2 = f(out, c)
    out = yield out2
    Return(out)
    
def main():
    def demo(s):
        run(eval(s), s)

    demo("f(1,2)")
    print
    demo("g(1,2)")
    print
    demo("h(1,2)")
    print
    demo("i(1,2,3)")

if __name__ == "__main__":
    main()

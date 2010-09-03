#! /usr/bin/env python
            
import traceback
import sys

class Return:
    def __init__(self, value):
        self.value = value
        raise self

indent = ""

def Run(f):
    f.next()
    m = f.send
    args = (None, )
    global indent
    while True:
        try:
            yield_val = m(*args)
        except StopIteration:
            raise Return(None)
        else:
            try:
                m = f.send
                if yield_val is None:
                    args = yield
                    args = (args, )
                else:
                    m = f.send
                    print "indent"
                    r = Run(yield_val)
                    val = None
                    while True:
                        indent += "  "
                        try:
                            r.send(val)
                            val = yield
                        except Return, r:
                            args = (r.value, )
                            break
                        except:
                            r.throw(*sys.exc_info())
                        finally:
                            indent = indent[2:]
            except:
                m = f.throw
                args = sys.exc_info()

def run(f, name):
    try:
        print indent, "Starting", name
        r = Run(f)
        r.send(None)
        while True:
            print indent, "Iterating on", name
            #r.throw(Exception("Hahaha!"))
            r.send(None)
    except Return, r:
        print indent, "Return value for %s:"%name, r.value
    except:
        print indent, "Had an exception in %s."%name
        traceback.print_exc()
        print indent, "Done with exception."

def f(a, b):
    print indent, "f started"
    yield
    print indent, "f running"
    Return(a + b)

def g(a, b):
    print indent, "g1"
    yield
    print indent, "g2"
    yield
    yield
    #raise Exception("Booh!")
    print indent, "g3"
    Return(5)

def h(a, b):
    print indent, "h1"
    yield
    print indent, "h2"
    out = yield (g(a, b))
    print indent, "h3"
    Return(out)

def i(a, b, c):
    print indent, "i started"
    yield
    print indent, "i running 1"
    out = yield (f(a, b))
    print indent, "i running 2", out 
    out2 = f(out, c)
    print indent, "i running 3", out2
    out = yield out2
    print indent, "i returning"
    Return(out)

run(f(1,2), "f")
print
run(g(1,2), "g")
print
run(h(1,2), "h")
print
run(i(1,2,3), "i")


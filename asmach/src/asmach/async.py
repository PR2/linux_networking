#! /usr/bin/env python
            
import traceback
import sys

class Return:
    def __init__(self, value):
        self.value = value
        raise self

def start(f):
    def run_internal(f):
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
                        r = run_internal(yield_val)
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
    
    r = run_internal(f)
    r.next()
    r.send(r)
    return r

def print_result(name, f = "<unnamed>"):
    yield
    try:
        out = yield f
        print name, "returned:", out
    except:
        print name, "had exception:"
        tb = list(sys.exc_info())
        tb[2] = tb[2].tb_next.tb_next
        traceback.print_exception(*tb)

def run(f, name = "<unnamed>"):
    try:
        r = start(print_result(name, f))
        while True:
            r.send(None)
    except Return, r:
        pass

def f(a, b):
    yield
    Return(a + b)

def g2():
    yield
    raise Exception("Booh!")

def g(a, b):
    yield
    yield
    yield g2()
    Return(5)

def h(a, b):
    yield
    out = yield (g(a, b))
    Return(out)

def i(a, b, c):
    yield
    out = yield (f(a, b))
    out2 = f(out, c)
    out = yield out2
    Return(out)

def demo(s):
    run(eval(s), s)

def main():
    demo("f(1,2)")
    print
    demo("g(1,2)")
    print
    demo("h(1,2)")
    print
    demo("i(1,2,3)")

if __name__ == "__main__":
    main()

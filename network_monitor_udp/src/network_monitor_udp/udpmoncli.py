#! /usr/bin/env python

from __future__ import with_statement

import time
import sys
import threading
import socket
import struct
import bisect
import traceback

class MonitorClient:
    def __init__(self, latencybins, destaddr, rate, pkt_length, paused = False, sourceaddr = None):
        self.mutex = threading.Lock()
        self.outstanding = {}
        self.arrived = []
        self.latencybins = list(latencybins)
        self.latencybins.sort()
        self.latencybins.append(1e1000)
        self.interval = 1.0 / rate
        self.send_thread = threading.Thread(target = self.send_thread_entry, name = "udpmoncli: send_thread")
        self.recv_thread = threading.Thread(target = self.recv_thread_entry, name = "udpmoncli: recv_thread")
        self.pkt_length = pkt_length
        self.exiting = False
        self.sourceaddr = self.resolve_addr(sourceaddr)
        self.destaddr = destaddr
        try:
            self.destaddr = self.resolve_addr(self.destaddr)
        except:
            pass # We'll try again later if it fails.
        self.lost = 0
 
        self._reset_bins()

        self.magic, = struct.unpack("=i", struct.pack("BBBB", 0xEF, 0x41, 0xC6, 0x35))
        self.pktstruct = "=iddi"
        self.hdr_len = struct.calcsize(self.pktstruct)
        if (pkt_length < self.hdr_len):
            print >> sys.stderr, "pkt_length must be at least", self.hdr_len
            return
        
        self.cv = threading.Condition()
        self.paused = True
        if not paused:
            self.start_monitor()

        self.window_start = time.time()
        self.recv_thread.start()
        self.send_thread.start()

    def resolve_addr(self, addr):
        if addr == None:
            return addr
        host, port = addr
        return (socket.gethostbyname(host), port)

    def init_socket(self, sourceaddr = None):
        if sourceaddr != None and self.sourceaddr != sourceaddr:
            self.sourceaddr = self.resolve_addr(sourceaddr)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.settimeout(0.2)
        if self.sourceaddr:
            self.socket.bind(self.sourceaddr)

    def start_monitor(self, sourceaddr = None):
        if self.paused:
            self.init_socket(sourceaddr)
            with self.cv:
               self.paused = False
               self.cv.notify_all()

    def stop_monitor(self):
        if not self.paused:
            self.paused = True
            self.socket.close()

    def _reset_bins(self):
         self.bins = [0 for i in range(0,len(self.latencybins))]

    def shutdown(self):
        print "Udp monitor starting to shut down"
        self.exiting = True
        with self.cv:
           self.cv.notify_all()

    def send_thread_entry(self):
        next_time = time.time()
        seqnum = 0
        while not self.exiting:
            try:
                sleeptime = next_time - time.time()
                if (sleeptime < -1):
                    print >> sys.stderr, "Send thread too far behind. Resetting expectations."
                    next_time = time.time()
                elif sleeptime > 0:
                    time.sleep(sleeptime)
                if self.paused:
                    with self.cv:
                        self.cv.wait()
                    next_time = time.time()
                    continue
                seqnum = seqnum + 1
                next_time = next_time + self.interval
                send_time = time.time()
                hdr = struct.pack(self.pktstruct, self.magic, send_time, 0, seqnum)
                with self.mutex:
                    self.outstanding[seqnum] = send_time
                try:
                    self.destaddr = self.resolve_addr(self.destaddr)
                except:
                    continue # This will count as a dropped packet. 
                self.socket.sendto(hdr.ljust(self.pkt_length), self.destaddr)
            except Exception, e:
                print "Got exception in send thread:", e
                traceback.print_exc(10)
                print self.destaddr
        print "Udp monitor send_thread finished shut down"

    def recv_thread_entry(self):
        while not self.exiting:
            if self.paused:
                with self.cv:
                    self.cv.wait()
                next_time = time.time()
                continue
            try:
                indata = self.socket.recv(4096)
                recv_time = time.time()
                (magic, send_time, echo_time, seq_num) = struct.unpack(self.pktstruct, indata[0:self.hdr_len])
                if magic != self.magic:
                    continue
                latency = recv_time - send_time
                self.bins[bisect.bisect(self.latencybins, latency)] += 1
                with self.mutex:
                    try:
                        del self.outstanding[seq_num]
                    except KeyError:
                        pass
                    self.arrived.append((send_time, latency))
            except socket.timeout:
                pass
        print "Udp monitor recv_thread finished shut down"

    def get_smart_bins(self, window):
        bins = [0 for i in range(0,len(self.latencybins))]
        with self.mutex: 
            now = time.time()
            arrived = self.arrived
            self.arrived = []
            outstanding = self.outstanding
            self.outstanding = {}
        window_end = now - self.latencybins[-2]
        window_start = self.window_start
        average = 0.
        count = 0
        average_restricted = 0.
        count_restricted = 0
        for pkt in arrived:
            (send_time, latency) = pkt
            if send_time < window_end:
                bins[bisect.bisect(self.latencybins, latency)] += 1
                count += 1
                average += latency
                if send_time >= window_start:
                    count_restricted += 1
                    average_restricted += latency
            else:
                self.arrived.append(pkt)
        missed = 0
        for (seq_num, send_time) in outstanding.iteritems():
            if send_time < window_end:
                missed += 1
                self.lost += 1
            else:
                self.outstanding[seq_num] = send_time
        if count == 0:
            count = 1
        if count_restricted == 0:
            count_restricted = 1
        count = float(count)
        count_restricted = float(count_restricted)
        bins = [ val / (count_restricted + missed) for val in bins ]
        self.window_start = window_end
        return bins, average / count, average_restricted / count_restricted

if __name__ == "__main__":
    try:
        if not len(sys.argv) in [5, 6]:
            print "usage: udpmoncli.py <host> <port> <pkt_rate> <pkt_size> [<src_addr>]"
            sys.exit(1)
        host = sys.argv[1]
        port = int(sys.argv[2])
        rate = float(sys.argv[3])
        size = int(sys.argv[4])
        if len(sys.argv) == 6:
            src_addr = sys.argv[5]
        else:
            src_addr = '0.0.0.0' 
        #cli = MonitorClient([.005, .01, .1, .2, .3, .4, .5], (host, int(port)), rate, size)
        cli = MonitorClient([.005, .01, .025, .05, .075, .1], (host, int(port)), rate, size) 
        try:
            display_interval = 0.5
            start_time = time.time()
            next_time = start_time
            while True:
                next_time = next_time + display_interval
                sleeptime = next_time - time.time()
                if sleeptime > 0:
                    time.sleep(sleeptime)
                if 0:
                    bins = cli.get_bins()
                else:
                    bins, average, average_restricted = cli.get_smart_bins(display_interval)
                print "%7.3f:"%(time.time() - start_time),
                for i in range(0,len(bins)):
                    print "%3i"%(int(100*bins[i])),
                    if i == 2:
                        print "  /",
                print "avg: %5.1f ms"%(1000*average), "avgr: %5.1f ms"%(1000*average_restricted), "loss: %6.2f %%"%(100 - 100 * sum(bins[0:-1]))
                sys.stdout.flush()
        finally:
            cli.shutdown()
            print >> sys.stderr, "Round trip latency summary (packets):", 
            for i in range(0, len(cli.latencybins)):
                print >> sys.stderr, "%.1f ms: %i before %i after"%(cli.latencybins[i] * 1000, sum(cli.bins[0:i+1]), sum(cli.bins[i+1:]) + cli.lost)

    except KeyboardInterrupt:
        print >> sys.stderr, "Exiting on CTRL+C."


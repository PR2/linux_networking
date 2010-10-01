import time
import state_publisher
import config
from twisted.internet import reactor
import mac_addr
import event
import multiset

class Frequency:
    def __init__(self, freq, period):
        self.freq = freq
        self.period = period
        self.next_scan_time_by_iface = {}
        self.next_scan_time = 0
        self.interfaces = multiset.Multiset()
        self.last_hit = 0

    def update_next_scan_time(self, iface, when):
        self.next_scan_time = when
        self.next_scan_time_by_iface[iface] = when

    def set_period(self, period):
        when = time.time() + period
        self.period = period
        self.next_scan_time = min(self.next_scan_time, when)
        for iface in self.next_scan_time_by_iface:
            self.next_scan_time_by_iface[iface] = min(self.next_scan_time_by_iface[iface], when)
    
class NoFrequenciesReady(Exception):
    def __init__(self, next_time):
        Exception.__init__(self)
        self.next_time = next_time

class FrequencyList:
    def __init__(self):
        self.frequencies = {}
        self.scan_period_cold = config.get_parameter('scan_period_cold', 20)
        self.scan_period_warm = config.get_parameter('scan_period_warm', 5)
        self.scan_period_hot = config.get_parameter('scan_period_hot', 2)

    def add(self, freq, iface):
        if freq not in self.frequencies:
            self.frequencies[freq] = Frequency(freq, self.scan_period_cold)
        self.frequencies[freq].interfaces.add(iface)

    def remove(self, freq, iface):
        self.frequencies[freq].interfaces.remove(iface)

    def next_scan_freq(self, iface, now = None, allow_early = False):
        if now == None:
            now = time.time()

        # Which frequencies can we consider?
        active_freqs = filter(lambda f: iface in f.interfaces, self.frequencies.itervalues())
        
        # Pick the frequency with the next expiry time.
        try:
            freq = min(active_freqs, key = lambda f: f.next_scan_time)
        except ValueError: # No active frequencies
            raise NoFrequenciesReady(None)
        
        # If no frequency has expired, take into account when it was last seen by this interface.
        if freq.next_scan_time > now:
            freq = min(active_freqs, key = lambda f: f.next_scan_time_by_iface.get(iface,0))
        
        if not allow_early and freq.next_scan_time > now:
            raise NoFrequenciesReady(freq.next_scan_time)
        freq.update_next_scan_time(iface, now + freq.period)
        return freq.freq

    def next_scan_freqs(self, iface, count, allow_early = False):
        now = time.time()
        freqs = []
        for i in range(count):
            try:
                f = self.next_scan_freq(iface, now, allow_early)
            except NoFrequenciesReady, nfr:
                if freqs:
                    break
                raise
            freqs.append(f)
        return freqs

    def set_freq_period(self, freq, period):
        f = self.frequencies[freq]
        f.set_period(period)

    def hit(self, freq, stamp):
        """Called each time a bss is seen."""
        try:
            self.frequencies[freq].last_hit = max(self.frequencies[freq].last_hit, stamp)
        except KeyError:
            print "Got a scan on an unexpected frequency." # FIXME
            pass # We got a hit on a frequency that we aren't scanning for. Strange.

class Bss:
    def __init__(self, bss):
        self.id = (bss.ssid, bss.bssid)
        self.ssid = bss.ssid
        self.bssid = bss.bssid
        self.by_iface = {}
        self.freq = bss.frequency
    
    def update(self, bss, iface):
        assert bss.ssid == self.ssid
        assert bss.bssid == self.bssid
        if self.freq != bss.frequency:
            print "Frequency for bss %s, %s has changed.", mac_addr.packed_to_str(bss.bssid), bss.ssid # FIXME
        self.by_iface[iface] = bss

    def last_seen(self, iface):
        if iface in self.by_iface:
            return self.by_iface[iface].stamp.to_sec()
        else:
            return 0

class BssList:
    def __init__(self, freq_list):
        self.bsses = {}
        self.freq_list = freq_list
        pass

    def update(self, bsses, iface):
        for bss in bsses:
            id = (bss.ssid, bss.bssid)
            if id not in self.bsses:
                self.bsses[id] = Bss(bss)
            self.bsses[id].update(bss, iface)
            self.freq_list.hit(bss.frequency, bss.stamp.to_sec())

        if False: # Gives an overview of currently known bsses.
            all = self.bsses.items()
            all.sort(key=lambda (x,y):(y.freq, x))
            now = time.time()
            print "\033[2J"
            print "\033[0;0H"
            for _, bss in all:
                print mac_addr.packed_to_str(bss.bssid), "%20.20s"%bss.ssid, bss.freq,
                ifaces = bss.by_iface.keys()
                ifaces.sort()
                min_stamp = now - max(bss.by_iface.itervalues(), key = lambda bss: bss.stamp).stamp.to_sec()
                max_level = max(bss.by_iface.itervalues(), key = lambda bss: bss.level).level
                print "%5.1f/%3i"%(min_stamp,max_level),
                for iface in ifaces:
                    print iface.name, "%5.1f/%3i"%(now - bss.by_iface[iface].stamp.to_sec(), bss.by_iface[iface].level),
                print
            print

            fl = self.freq_list.frequencies.keys()
            fl.sort()
            for f in fl:
                fc = self.freq_list.frequencies[f]
                if fc.last_hit:
                    print f, "%5.1f"%(now - fc.last_hit),
                else:
                    print f, "never",
                print "%5.1f"%(fc.next_scan_time - now),
                for iface in fc.next_scan_time_by_iface:
                    print "%s %5.1f"%(iface.name, now - fc.next_scan_time_by_iface[iface]),
                print
            print

class ScanManager:
    def __init__(self):
        self.scan_results = {}
        self.frequencies = FrequencyList()
        self.num_scan_frequencies = config.get_parameter('num_scan_frequencies', 4)
        self.scheduled_scan = None
        self.bss_list = BssList(self.frequencies)
        self.new_scan_data = event.Event()

    def add_iface(self, iface):
        iface.radio_sm.scanning.subscribe(self._scanning_state_cb, iface)
        iface.radio_sm.scan_event.subscribe_repeating(self._scan_event_cb, iface)
        iface.radio_sm.scanning_enabled.subscribe(self._scanning_state_cb, iface)
        iface.radio_sm.frequency_list.subscribe(self._frequency_list_cb, iface)

    def _scanning_state_cb(self, iface, old_state, new_state):
        """A state change that might cause us to start scanning has occurred."""
        self._trigger_scan(iface)

    def _trigger_scan(self, iface):
        #print "_trigger_scan", iface.name 
        if self.scheduled_scan:
            if self.scheduled_scan.active():
                self.scheduled_scan.cancel()
            self.scheduled_scan = None
        if not iface.radio_sm.scanning_enabled.get() or iface.radio_sm.scanning.get():
            return # We are not in a scanning state, or we are currently scanning.
        try:
            freqs = self.frequencies.next_scan_freqs(iface, self.num_scan_frequencies, not iface.radio_sm.associated.get())
        except NoFrequenciesReady, nfr:
            if nfr.next_time:
                self.scheduled_scan = reactor.callLater(max(0.1, nfr.next_time - time.time()), self._trigger_scan, iface)
        else:
            iface.radio_sm.scan(freqs)
            #print "Triggered scan", iface.name, freqs

    def _scan_event_cb(self, iface, bsses):
        self.bss_list.update(bsses, iface)
        self.new_scan_data.trigger()

    def _frequency_list_cb(self, iface, old_state, new_state):
        now = time.time()
        for f in new_state:
            self.frequencies.add(f, iface)
        
        if old_state != state_publisher.JustSubscribed:
            for f in old_state:
                self.frequencies.remove(f, iface)

        self._trigger_scan(iface)

class RadioManager:
    def __init__(self):
        self.scan_manager = ScanManager()
        self.initial_inhibit_end = time.time() + config.get_parameter('initial_assoc_inhibit', 5)
        self.bss_expiry_time = config.get_parameter('bss_expiry_time', 5)
        self.scan_manager.new_scan_data.subscribe_repeating(self._new_scan_data)
        self.interfaces = set()

    def add_iface(self, iface):
        self.scan_manager.add_iface(iface)
        self.interfaces.add(iface)

    def _new_scan_data(self):
        print "_new_scan_data"
        now = time.time()
        if now < self.initial_inhibit_end:
            print "inhibited"
            return
        for iface in self.interfaces:
            if iface.radio_sm.scanning_enabled.get():
                # Pick the best bss for this interface.
                candidate_bsses = filter(lambda bss: bss.last_seen(iface) > now - self.bss_expiry_time, 
                        self.scan_manager.bss_list.bsses.itervalues())
                best_bss = max(candidate_bsses, key = lambda bss: bss.by_iface[iface].level)

                # If we are associated, do we want to switch?
                cur_assoc = iface.radio_sm.associated.get()
                print cur_assoc
                if cur_assoc:
                    break
                    #if cur_assoc.level + 15 > best_bss.level:
                    #    break

                # Let's associate
                print "associating", iface.name, "to", mac_addr.packed_to_str(best_bss.bssid), best_bss.ssid
                iface.radio_sm.associate_request.trigger(best_bss.id)



        


from __future__ import with_statement

import radio
import threading
import time           
from heapq import heappush, heappop

STATE_UNASSOCIATED = 0
STATE_ASSOCIATING = 1
STATE_ASSOCIATED = 2
STATE_ACTIVE = 3

FREQUENCY_

## @brief Makes association and scanning decisions for a radio.
##
## A RadioManager is associated with a Radio. It is in charge of deciding
## when to scan, what frequency to scan, what to associate to, and when to
## change associations. It uses a NetworkSelector to help it decide which
## BSSID to connect to.
class RadioManager:
    def __init__(self, interface_name, state_change_callback):
        self.mutex = threading.Lock
        self.scan_queue = []
        self.scan_intervals = {}
        with self.mutex:
            self.association = None
        
            self.radio = radio.Radio(interface_name, associate_callback = _associated,
                unassociate_callback = _unassociated, scan_callback = _scan_results_cb,
                scan_done_callback = _scan_done_cb, frequency_list_callback = _freq_cb):

            self.set_state(STATE_UNASSOCIATED)
    
    def _pick_scan_freq(self):
        assert self.mutex.locked()
        (_, freq) = heappop(self.scan_queue)
        heappush(self.scan_queue, time.time() + self.scan_intervals[freq])
        return freq

    def _try_trigger_scan(self): 
        assert self.mutex.locked()
        if state == STATE_ACTIVE || state == STATE_ASSOCIATING:
            self.radio.cancel_scans()
        elif not self.radio.is_scanning():
            #@todo Do hidden networks, and more than one freq/ssid at once.
            self.radio.scan([_pick_scan_freq()])

    def _set_state(self, state): 
        assert self.mutex.locked()
        self.state = state
        _try_trigger_scan() # Cancels or starts scanning as necessary
    
    def failed(self, level):
        with self.mutex:
            self.radio.unassociate()
            self._set_state(self, STATE_UNASSOCIATED)

    def connected(self):
        with self.mutex:
            pass

    def activate(self):
        with self.mutex:
            if self.state == STATE_ASSOCIATED:
                self._set_state(STATE_ACTIVE)

    def deactivate(self):
        with self.mutex:
            if self.state == STATE_ACTIVE:
                self._set_state(STATE_ASSOCIATED)
    
    def _freq_cb():
        with self.mutex:
            new_freqs = self.robot.frequencies
            old_queue = self.scan_queue
            old_freqs = set()
            self.scan_queue = 0
            for t, f in old_queue: # Copy existing freqs to queue
                old_freqs.add(f)
                if f in new_freqs:
                    heappush(self.scan_queue, (t, f))

            # Blaise was here.

            _try_trigger_scan()

    def _scan_done_cb():
        with self.mutex:
            self._try_trigger_scan()

    def _scan_results_cb():
        with self.mutex:
            pass

    def _unassociated(self, radio):
        with self.mutex:
            if state == STATE_ASSOCIATING:
                self.selector.failed(self.association, FAILED_ASSOCIATION) #FIXME
            self._set_state(STATE_UNASSOCIATED)

    def _associated(self, radio):
        with self.mutex:
            self.association = (radio.association.ssid, radio.association.bssid)
            self._set_state(STATE_ASSOCIATED)

    

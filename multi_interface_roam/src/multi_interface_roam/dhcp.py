#! /usr/bin/env python

import roslib; roslib.load_manifest('multi_interface_roam')
import asmach as smach
import scapy.all as scapy
from twisted.internet.defer import inlineCallbacks, returnValue
from netlink_monitor import monitor, IFSTATE
import async_helpers
import l2socket
import event

class DhcpData:
    def __init__(self, iface):
        self.iface = iface
        self.socket = None
        self.link_addr_state = monitor.get_state_publisher(self.iface, IFSTATE.LINK_ADDR) 
        self.error_event = event.Event()
        self.error_timeout = 60
        self.exp_backoff_min = 0.2
        self.exp_backoff_max = 0.5
        self.exp_backoff_timeout = 2

    def start_socket(self):
        if not self.socket or self.socket.port.fileno() == -1:
            self.socket = async_helpers.ReadDescrEventStream(l2socket.L2Port, iface = self.iface, 
                    filter='udp and dst port 68 and src port 67')

    def stop_socket(self):
        self.socket = None

class DhcpState(smach.State):
    def __init__(self, *args, **kwargs):
        smach.State.__init__(self, input_keys=['dhcp'], output_keys=['dhcp'], *args, **kwargs)



class ExchangeRetryExponentialBackoff:
    def init_retry_gen(self, ud):
        self.cur_retry_max = ud.exp_backoff_min

    def get_next_retry(self, ud):
        interval = self.cur_retry_max * random.uniform(0.5, 1)
        self.cur_retry_max = min(ud.exp_backoff_max, 2 * self.cur_retry_max)



class ExchangeRetryHalve:
    def init_retry_gen(self, ud):
        pass

    def get_next_retry(self, ud):
        interval = 60 + (ud.timeout_time[self.type] - time.time()) / 2


        
class ExchangeDiscover:
    def send(self, ud):
        hwbytes = scapy.mac2str(self.ud.hwaddr)
        pkt = (
              scapy.Ether(src=self.ud.hwaddr, dst='ff:ff:ff:ff:ff:ff')/
              scapy.IP(src='0.0.0.0', dst='255.255.255.255')/
              scapy.UDP(sport=68, dport=67)/
              scapy.BOOTP(chaddr=[hwbytes], xid=self.xid)/
              scapy.DHCP(options=[
                  ("message-type", "discover"),
                  "end",
                  ])
              )
        self.socket.port.send(str(pkt))

    def validate(self, pkt, ud):



class ExchangeRequest:
    def send(self, ud):
        hwbytes = scapy.mac2str(self.ud.hwaddr)
        pkt = (
              scapy.Ether(src=self.ud.hwaddr, dst='ff:ff:ff:ff:ff:ff')/
              scapy.IP(src='0.0.0.0', dst='255.255.255.255')/
              scapy.UDP(sport=68, dport=67)/
              scapy.BOOTP(chaddr=[hwbytes], xid=self.xid)/
              scapy.DHCP(options=[
                  ("message-type", "request"),
                  "end",
                  ])
              )
        self.socket.port.send(str(pkt))

    def validate(self, pkt, ud):
        Set ud.timeout_time



class NoLink(DhcpState):
    def __init__(self):
        DhcpState.__init__(self, outcomes=['bound', 'init', 'init_reboot'])
 
    @inlineCallbacks
    def execute_async(self, ud):
        ud.dhcp.hwaddr = yield async_helpers.wait_for_state(ud.dhcp.link_addr_state, False, True)
        returnValue('init')



class Init(DhcpState):
    def __init__(self):
        DhcpState.__init__(self, outcomes=['done', 'nolink'])

    @inlineCallbacks 
    def execute_async(self, ud):
        ud.dhcp.start_socket()
        returnValue('done')



class Exchange(DhcpState):
    def __init__(self, mode):
        DhcpState.__init__(self, outcomes=['success', 'fail', 'nolink'])
        self.mode = mode
 
    @inlineCallbacks
    def execute_async(self, ud):
        # Parameters that will depend on the state.
        self.init_retry_gen(ud)
        timeout = Timeout()

        # Make sure we aren't discarding incoming dhcp packets.
        ud.dhcp.socket.set_discard(False)

        # Generate an xid for the exchange.
        self.xid = random.randint(0, 0xFFFF)

        while True:
            # Send a request packet
            try:
                self.send_time = time.time()
                self.send(ud)
                FIXME
                ### discover/request
                ### 
            except:
                returnValue('fail')

            # How long to wait before retry
            self.get_next_retry(ud)

            while True:
                # Wait for an event
                events = yield async_helpers.select(
                        StateCondition(ud.dhcp.link_addr_state, False, False), 
                        ud.dhcp.socket, 
                        Timeout(interval),
                        timeout)

                if 0 in events: # Lost link
                    returnValue('nolink')
    
                if 1 in events: # Got packet
                    pkt = Ether(ud.dhcp.socket.recv())
                    pkt = pkt.payload # IP
                    pkt = pkt.payload # UDP
                    pkt = pkt.payload # BOOTP

                    # Check xid
                    # Check this is a response packet
                    # Check it is destined to us
                    valid = self.validate(ud, pkt)
                    if valid:
                        returnValue(valid)
    
                if 2 in events:
                    break
    
                if 3 in events:
                    returnValue('fail')



class Rebooting  (Exchange, ExchangeRetryExponentialBackoff, ExchangeRequest ): type = "Rebooting"
class Selecting  (Exchange, ExchangeRetryExponentialBackoff, ExchangeDiscover): type = "Selecting"
class Requesting (Exchange, ExchangeRetryExponentialBackoff, ExchangeRequest ): type = "Requesting"
class Renewing   (Exchange, ExchangeRetryHalve,              ExchangeRequest ): type = "Renewing"
class Rebinding  (Exchange, ExchangeRetryHalve,              ExchangeRequest ): type = "Rebinding"



class Error(DhcpState):
    def __init__(self):
        DhcpState.__init__(self, outcomes=['done', 'nolink'])
 
    @inlineCallbacks
    def execute_async(self, ud):
        ud.dhcp.error_event.trigger()
        ud.dhcp.socket.set_discard(True)
        events = yield async_helpers.switch({
            StateCondition(ud.dhcp.link_addr_state, False, False) : lambda _: returnValue('nolink'), 
            Timeout(ud.dhcp.error_timeout)                        : lambda _: returnValue('timeout'),
            })



class Bound(DhcpState):
    def __init__(self):
        DhcpState.__init__(self, outcomes=['timeout', 'nolink'])
 
    @inlineCallbacks
    def execute_async(self, ud):
        ud.dhcp.socket.set_discard(True)
        FIXME Actually set the address
        events = yield async_helpers.switch({
            StateCondition(ud.dhcp.link_addr_state, False, False) : lambda _: returnValue('nolink'), 
            Timeout(ud.timeout_time['Bound'] - time.time())       : lambda _: returnValue('timeout'),
            })



def start_dhcp(iface):
    sm = smach.StateMachine(outcomes=[], input_keys=['dhcp'])
    smadd = smach.StateMachine.add
    with sm:
        smadd('NOLINK',      NoLink(),      transitions = {'bound'  :'BOUND',      'init':'INIT',      'init_reboot':'INIT_REBOOT'})
        smadd('INIT_REBOOT', Init(),        transitions = {'done'   :'REBOOTING',                      'nolink'     :'NOLINK'})
        smadd('REBOOTING',   Rebooting(),   transitions = {'success':'BOUND',      'fail':'INIT',      'nolink'     :'NOLINK'})
        smadd('INIT',        Init(),        transitions = {'done'   :'SELECTING',                      'nolink'     :'NOLINK'})
        smadd('SELECTING',   Selecting(),   transitions = {'success':'REQUESTING', 'fail':'ERROR',     'nolink'     :'NOLINK'})
        smadd('REQUESTING',  Requesting(),  transitions = {'success':'BOUND',      'fail':'ERROR',     'nolink'     :'NOLINK'})
        smadd('BOUND',       Bound(),       transitions = {'timeout':'RENEWING',                       'nolink'     :'NOLINK'})
        smadd('RENEWING',    Renewing(),    transitions = {'success':'BOUND',      'fail':'REBINDING', 'nolink'     :'NOLINK'})
        smadd('REBINDING',   Rebinding(),   transitions = {'success':'BOUND',      'fail':'INIT',      'nolink'     :'NOLINK'})
        smadd('ERROR',       Error(),       transitions = {'done'   :'INIT',                           'nolink'     :'NOLINK'})

    ud = smach.UserData()
    ud.dhcp = DhcpData(iface)
    def shutdown(value):
        reactor.fireSystemEvent('shutdown')
    def ignore_eintr(error):
        if error.type == IOError:
            import errno
            if error.value.errno == errno.EINTR:
                return None
        return error
    sm.execute_async(ud).addCallback(shutdown).addErrback(ignore_eintr)

if __name__ == "__main__":
    import sys
    from twisted.internet import reactor

    if len(sys.argv) != 2:
        print "usage: dhcp.py <interface>"
        sys.exit(1)

    iface = sys.argv[1]

    start_dhcp(iface)
    reactor.run()    

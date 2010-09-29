from __future__ import with_statement
import radio
from netlink_monitor import netlink_monitor, IFSTATE
import async_helpers

class RadioManagerData:
    def __init__(self, iface):
        self.iface = iface
        self.up_state_pub = netlink.get_state_pub(IFSTATE.UP)
        self.activate_request = state_publisher.StatePublisher()
        self.disactivate_delay = 1

class RadioManagerState(smach.State):
    def __init__(self, *args, **kwargs):
        smach.State.__init__(self, input_keys=['radio'], output_keys=['radio'], *args, **kwargs)

class Down(RadioManagerState):
    def __init__(self):
        RadioManagerState.__init__(self, outcomes=['up'])
    
    @inlineCallbacks
    def execute_async(self, ud):
        yield async_helpers.wait_for_state(ud.radio.up_state_pub, lambda x: operator.__not__)
        returnValue('up')

class Unassociated(RadioManagerState):
    def __init__(self):
        RadioManagerState.__init__(self, outcomes=['associate', 'down'])
    
    @inlineCallbacks
    def execute_async(self, ud):
        # If we got here then were forcibly disactivated, and the previous
        # activation request is moot.
        ud.radio.activate_request.set(False)

        # We may have landed here after the interface went down causing
        # disassociation. Quickly check for that before getting to work.
        if not ud.radio.up_state_pub.get():
            returnValue('down')
        
        pass

class Associating(RadioManagerState):
    def __init__(self):
        RadioManagerState.__init__(self, outcomes=['associated', 'failed', 'down'])
    
    @inlineCallbacks
    def execute_async(self, ud):
        events = yield async_helpers.select(
                async_helpers.StateCondition(ud.radio.up_state_pub, lambda x: operator.__not__ ),
                async_helpers.StateCondition(ud.radio.associated, lambda x: x != radio.Associating))

        if 0 in events:
            returnValue('down')

        elif ud.radio.associated.get() == radio.Unassociated:
            returnValue('failed')
        else:
            returnValue('associated')

class Associated(RadioManagerState):
    def __init__(self):
        RadioManagerState.__init__(self, outcomes=['activate', 'reassociate', 'unassociated'])
    
    @inlineCallbacks
    def execute_async(self, ud):
        pass

class Active(RadioManagerState):
    def __init__(self):
        RadioManagerState.__init__(self, outcomes=['disactivate', 'unassociated'])
    
    @inlineCallbacks
    def execute_async(self, ud):
        events = yield async_helpers.select(
          async_helpers.StateCondition(ud.radio.activate_request, operator.__not__),
          async_helpers.StateCondition(ud.radio.associated, operator.__not__))
        if 1 in events:
            returnValue('unassociated')
        returnValue('disactivate')

class Disactivating(RadioManagerState)
    def __init__(self):
        RadioManagerState.__init__(self, outcomes=['done', 'activate', 'unassociated'])

    @inlineCallbacks
    def execute_async(self, ud):
        events = yield async_helpers.select(
          async_helpers.Timeout(ud.radio.disactivate_delay),
          async_helpers.StateCondition(ud.radio.activate_request, operator.truth),
          async_helpers.StateCondition(ud.radio.associated, operator.__not__),
        if 2 in events:
            returnValue('unassociated')
        if 1 in events:
            returnValue('activate')
        returnValue('done')

def radio_manager(iface):
    sm = smach.StateMachine(outcomes=[], input_keys=['radio'])
    smadd = smach.StateMachine.add
    with sm:
        smadd('DOWN', Down(),  transitions = { 'up' : 'UNASSOCIATED' })
        smadd('UNASSOCIATED', Unassociated(), transitions = { 'associate' : 'ASSOCIATING', 'down' : 'DOWN'})
        smadd('ASSOCIATING', Associating(), transitions = { 'associated' : 'ASSOCIATING', 'failed' : 'DOWN'})
        smadd('ASSOCIATED', Associated(), transitions = { 'activate' : 'ACTIVE', 'unassociated' : 'UNASSOCIATED'})
        smadd('ACTIVE', Active(), transitions = { 'disactivate' : 'DISACTIVATING', 'unassociated' : 'UNASSOCIATED'} )
        smadd('DISACTIVATING', Disctivating(), transitions = { 'done' : 'ASSOCIATED', 'activate' : 'ACTIVE', 'unassociated' : 'UNASSOCIATED'} )

    ud = smach.UserData()
    ud.radio = RadioManagerState(iface)
    sm.execute_async(ud)
    return ud.radio

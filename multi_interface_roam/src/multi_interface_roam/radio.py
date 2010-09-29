from __future__ import with_statement
import roslib; roslib.load_manifest("multi_interface_roam")
import rospy
import wpa_msg as wpa_msgs
import actionlib                       
import state_publisher
import event
from twisted.internet.reactor import reactor

class Network:
    def __init__(id, enabled, parameters):
        self.id = id
        self.enabled = enabled
        if type(parameters) != dict:
            # assume it is a message
            parameters = dict((p.key, p.value) for p in parameters)
        self.parameters = parameters

class AssociationState:
    def __nonzero__():
        return False

Associating = AssociationState()
Unassociated = AssociationState()

class Radio:
    def __init__(interface_name)
        self.interface_name = interface_name
        self.associated = state_publisher.StatePublisher(False)
        self.frequency_list = state_publisher.StatePublisher([])
        self.network_list = state_publisher.StatePublisher([])
        self.scan_event = event.Event()
        self.scanning_done_event = event.Event()

        self.networks = []
        self.hidden_networks = []
        self.raw_scan_results = []
        self.scan_results = []

        prefix = rospy.resolve_name("wpa_supplicant")+"/"+interface_name+"/"
        rospy.Subscriber(prefix + "association_state", wpa_msgs.AssociateFeedback, _association_state_callback)
        rospy.Subscriber(prefix + "scan_results", wpa_msgs.ScanResult, _scan_results_callback)
        rospy.Subscriber(prefix + "network_list", wpa_msgs.NetworkList, _network_list_callback)
        rospy.Subscriber(prefix + "frequency_list", wpa_smsg.FrequencyList, _frequency_list_callback)
        self._scan_action actionlib.SimpleActionClient(prefix+"/scan", wpa_msg.ScanAction)
        self._associate_action actionlib.SimpleActionClient(prefix+"/associate", wpa_msg.AssociateAction)

    def is_scanning(self):
        return _scan_action.simple_state == SimpleGoalState.DONE
    
    def scan(self, freqs = [], ssids = []):  
        goal = wpa_supplicant_node.msg.ScanGoal()
        for s in ssids:
            goal.ssids.append(s)
            for f in freqs:
                goal.frequencies.append(f)
        _scan_action.send_goal(goal, done_cb = _scan_done_callback)

    def cancel_scans(self):
        self.scanning = False
        _scan_action.cancel_all_goals()

    def associate(ssid, bssid):
        _associate_action.send_goal(wpa_msg.AssociateGoal(ssid, bssid))
        self.associated.set(Associating)

    def unassociate():
        _associate_action.cancel_all_goals()

    ## @brief Called when all the scans we have initiated are done.
    def _scan_done_callback(self, rslt):
        self.scanning_done_event.trigger()

    def _frequency_list_callback(self, msg):
        self.frequency_list(msg)

    def _network_list_callback(self, msg):
        self.networks = [ Network(net.network_id, net.enabled, net.parameters) for net in msg.networks ]
        self.hidden_networks = [ net for net in self.networks if net.parameters['scan_ssid'] == '1' ]
        self.network_list_callback.trigger()
        if self.network_list_callback:
            self.network_list_callback(self)
        self._filter_raw_scan_results() # The filtered scan results are out of date.

    def enabled_bss(bss):
        for net in self.networks:
            # @todo This check will not work for <any> netwoks.
            # @todo Might want to check that security parameters match too.
            if net['ssid'] == bss.ssid:
                return True
        return False

    def _filter_raw_scan_results(self, msg)
        self.scan_results = [ bss for bss in self.raw_scan_results if enabled_bss(bss) ]
        self.scan_event.trigger(self.scan_results)
    
    def _scan_results_callback(self, msg):
        self.raw_scan_results = msg.bss
        self._filter_raw_scan_results() 

    def _associate_state_callback(self, msg):
        if msg.associated:
            self.associated.set(msg.bss)
        else:
            self.associated.set(self.Unassociated)

#! /usr/bin/env python
import roslib; roslib.load_manifest('wpa_supplicant_node')
import rospy
import actionlib
import wpa_supplicant_node.msg
from actionlib_msgs.msg import GoalStatus

def bssid_to_str(bssid):
    return ":".join(["%02X"%ord(c) for c in bssid])

class Test:
    def __init__(self, iface):
        self.assoc_tries = 0
        self.assoc_successes = 0
        self.scan_action = actionlib.SimpleActionClient(iface+"/scan", wpa_supplicant_node.msg.ScanAction)
        rospy.loginfo("Waiting for scan server.")
        self.scan_action.wait_for_server()
        
        self.assoc_action = actionlib.SimpleActionClient(iface+"/associate", wpa_supplicant_node.msg.AssociateAction)
        rospy.loginfo("Waiting for associate server.")
        self.assoc_action.wait_for_server()

    def scan(self, ssid = [], freq = []):
        goal = wpa_supplicant_node.msg.ScanGoal()
        for s in ssid:
            goal.ssids.append(s)
        for f in freq:
            goal.frequencies.append(f)
        
        start = rospy.get_time()
        rospy.loginfo("Sending goal.")
        self.scan_action.send_goal(goal)
        rospy.loginfo("Waiting for result.")
        self.scan_action.wait_for_result()
        end = rospy.get_time()
        
        rospy.loginfo("Scan took %.3f seconds."%(end-start))
        return self.scan_action.get_result()  
    
    def associated(self, feedback):
        if feedback.associated:
            end = rospy.get_time()
            rospy.loginfo("Association took %.3s seconds."%(end - self.assoc_start))
            self.assoc_count += 1
    
    def associate(self, bss):
        self.assoc_start = rospy.get_time()
        self.assoc_count = 0                                 
        self.assoc_tries += 1

        rospy.loginfo("Associating to %s:"%bssid_to_str(bss.bssid))
        self.assoc_action.send_goal(wpa_supplicant_node.msg.AssociateGoal(bss), feedback_cb=self.associated)
        failed = False
        while not self.assoc_count:
            state = self.assoc_action.get_state() 
            if state == GoalStatus.ABORTED or state == GoalStatus.REJECTED:
                failed = True
                break
            rospy.sleep(rospy.Duration(0.1))

        if not failed:
            #rospy.sleep(rospy.Duration(2.2))
    
            self.assoc_action.cancel_goal()
            rospy.sleep(rospy.Duration(1))
    
            if self.assoc_count > 1:
                rospy.logerr("Got multiple association feedbacks.")
            
            if self.assoc_count == 1:
                self.assoc_successes += 1
                return True

        return False

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('test_scan')
        iface = 'wlan0'
        t = Test(iface)                  
        while True:
            result = t.scan([], [2437, 5240])
            #print result
            for i in range(0,5):
                for bss in result.bss:
                    if rospy.is_shutdown():
                        raise KeyboardInterrupt
                    #if bssid_to_str(bss.bssid) == "00:24:6C:81:D5:E8":
                    if bss.ssid == 'blaise-test': 
#                    if bssid_to_str(bss.bssid) == "00:24:6C:81:D5:E8" or \
#                      bss.ssid == 'blaise-test': 
                        print "Network found. Associating... %s %s %i"%(bss.ssid, bssid_to_str(bss.bssid), bss.frequency)
                        t.associate(bss)
                        #if not t.associate(bss):
                        #    raise KeyboardInterrupt
                        #while t.associate(bss):
                        #    pass
                        print "Successes: %i/%i"%(t.assoc_successes, t.assoc_tries)
                        if t.assoc_tries >= 1000:
                            raise KeyboardInterrupt
                    #else:
                    #    print "skipping", bssid_to_str(bss.bssid)

    except KeyboardInterrupt:
        print "program interrupted before completion"
    


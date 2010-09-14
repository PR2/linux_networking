#! /usr/bin/env python
import roslib; roslib.load_manifest('wpa_supplicant_node')
import rospy
import actionlib
import wpa_supplicant_node.msg
from actionlib_msgs.msg import GoalStatus
import os

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
            
        print "Requesting scan..."
        rospy.sleep(rospy.Duration(1))
        start = rospy.get_time()
        #rospy.loginfo("Sending goal.")
        self.scan_action.send_goal(goal)
        #print "Requesting scan..."
        #rospy.loginfo("Waiting for result.")
        self.scan_action.wait_for_result()
        end = rospy.get_time()
        
        rospy.loginfo("Scan took %.3f seconds."%(end-start))
        return self.scan_action.get_result()  
    
    def associated(self, feedback):
        if feedback.associated:
            end = rospy.get_time()
            print "Association took %.3f seconds."%(end - self.assoc_start),
            self.assoc_count += 1
    
    def associate(self, bss):
        self.assoc_start = rospy.get_time()
        self.assoc_count = 0                                 
        self.assoc_tries += 1

        #rospy.loginfo("Associating to %s:"%bssid_to_str(bss.bssid))
        print "Associating to %12s %s %4i"%(bss.ssid, bssid_to_str(bss.bssid), bss.frequency),
        sys.stdout.flush()
        self.assoc_action.send_goal(wpa_supplicant_node.msg.AssociateGoal(bssid = bss.bssid, ssid = bss.ssid), feedback_cb=self.associated)
        failed = False
        while not self.assoc_count:
            state = self.assoc_action.get_state() 
            if state == GoalStatus.ABORTED or state == GoalStatus.REJECTED:
                failed = True
                break
            rospy.sleep(rospy.Duration(0.1))

        rospy.sleep(rospy.Duration(0.5))
        
        if not failed:
            #rospy.sleep(rospy.Duration(2.2))
    
            #os.system('ifconfig %s down'%iface)
            #os.system('ifconfig %s up'%iface)
            self.assoc_action.cancel_goal()
            rospy.sleep(rospy.Duration(0.25))
    
            if self.assoc_count > 1:
                print
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
        import sys
        iface = sys.argv[1]
        t = Test(iface)              
        count = 0
        while not rospy.is_shutdown():
            count += 1
            #if count % 2:
            #    result = t.scan([], [5805])
            #else:
            #    result = t.scan([], [2437])
            #result = t.scan([], [])
            #result = t.scan([], [5260, 5805])
            result = t.scan([], [5260, 5805, 2437])
            #if count > 1:
            print len(result.bss)
            #    rospy.sleep(rospy.Duration(0.5))
            #    continue
            #rospy.sleep(rospy.Duration(0.2))
            #continue
            #for b in result.bss:
            #    print "%s %s"%(bssid_to_str(b.bssid), b.ssid)
            #continue
            #result = t.scan([], [2437, 2462, 5240, 5260, 5805])
            #if not result.bss:
            #    raise Exception("No scan output!")
            #print result
            for i in range(0,5):
                for bss in result.bss:
                    if rospy.is_shutdown():
                        raise KeyboardInterrupt
                    bssidlist = [
                      #"00:24:6C:81:4E:FA", # willow-wpa2
                      #"00:24:6C:81:D5:E0", # willow
                      #"00:24:6C:81:D5:EA", # willow-wpa2
                      #"00:24:6C:81:D5:E8", # willow blaise-office
                      "00:24:6C:82:4E:F8", # willow lab
                      ]
                    essidlist = [
                      #"willow-wpa2",
                      "blaise-test",
                      #"willow",
                      ]
                    if bssid_to_str(bss.bssid) in bssidlist or \
                       bss.ssid in essidlist:
                        #print "Network found. Associating... %s %s %i"%(bss.ssid, bssid_to_str(bss.bssid), bss.frequency)
                        t.associate(bss)
                        #if not t.associate(bss):
                        #    raise KeyboardInterrupt
                        #while t.associate(bss):
                        #    pass
                        print "Successes: %i/%i"%(t.assoc_successes, t.assoc_tries)
                        sys.stdout.flush()
                        #if t.assoc_tries >= 1000:
                        #    raise KeyboardInterrupt
                    #else:
                    #    print "skipping", bssid_to_str(bss.bssid)

    except KeyboardInterrupt:
        print "program interrupted before completion"
    


#! /usr/bin/env python
import roslib; roslib.load_manifest('wpa_supplicant_node')
import rospy
import actionlib
import wpa_supplicant_node.msg

def scan(iface = 'wlan0', ssid = [], freq = []):
    client = actionlib.SimpleActionClient(iface+"/scan", wpa_supplicant_node.msg.ScanAction)
    rospy.loginfo("Waiting for server.")
    client.wait_for_server()

    goal = wpa_supplicant_node.msg.ScanGoal()
    for s in ssid:
        goal.ssids.append(s)
    for f in freq:
        goal.frequencies.append(f)
    
    rospy.loginfo("Sending goal.")
    client.send_goal(goal)
    rospy.loginfo("Waiting for result.")
    client.wait_for_result()
    return client.get_result()  

def associate(iface, bss):
    client = actionlib.SimpleActionClient(iface+"/associate", wpa_supplicant_node.msg.AssociateAction)
    rospy.loginfo("Waiting for server.")
    client.wait_for_server()

    rospy.loginfo("Sending goal.")
    client.send_goal(wpa_supplicant_node.msg.AssociateGoal(bss))
    rospy.loginfo("Waiting for result.")
    client.wait_for_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('test_scan')
        iface = 'wlan0'
        result = scan(iface, ['willow'])
        print result
        for bss in result.bss:
            if bss.ssid == 'willow':
                print "Willow found. Associating..."
                associate('wlan0', bss)
                break;

    except KeyboardInterrupt:
        print "program interrupted before completion"
    


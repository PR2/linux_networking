#! /usr/bin/env python
import roslib; roslib.load_manifest('wpa_supplicant_node')
import rospy
import actionlib
import wpa_supplicant_node.msg

def scan(iface = 'wlan0', ssid = [], freq = []):
    client = actionlib.SimpleActionClient(iface, wpa_supplicant_node.msg.ScanAction)
    rospy.loginfo("Waiting for server.")
    client.wait_for_server()

    goal = wpa_supplicant_node.msg.ScanGoal()
    print goal.ssids
    for s in ssid:
        goal.ssids.append(s)
    for f in freq:
        goal.frequencies.append(f)
    print goal.ssids
    rospy.loginfo("Sending goal.")
    client.send_goal(goal)
    rospy.loginfo("Waiting for result.")
    client.wait_for_result()
    return client.get_result()  

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('test_scan')
        result = scan(ssid=['willow'], freq = [5180])
        print result
    except KeyboardInterrupt:
        print "program interrupted before completion"
    


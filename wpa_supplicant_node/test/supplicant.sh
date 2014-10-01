#! /bin/sh
. `dirname $0`/config.sh
rosrun wpa_supplicant_node wpa_supplicant_node -Dnl80211 -dddd -c ~/temp/net.conf -i $WLAN_TEST

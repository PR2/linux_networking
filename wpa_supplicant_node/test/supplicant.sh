#! /bin/sh
. `dirname $0`/config.sh
./bin/wpa_supplicant_node -Dnl80211 -dddd -c ~/temp/net.conf -i $WLAN_TEST

#! /bin/sh
. `dirname $0`/config.sh
iw dev $WLAN_MONI interface add $MONI_MONI type monitor
ifconfig $MONI_MONI up
iw dev $WLAN_MONI set freq $FREQ
tcpdump -i $MONI_MONI -n -l -e|grep $MAC_TEST --color


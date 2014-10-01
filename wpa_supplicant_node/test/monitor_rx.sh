#! /bin/sh
. `dirname $0`/config.sh
iw dev $WLAN_TEST interface add $MONI_TEST type monitor
ifconfig $MONI_TEST up
tcpdump -i $MONI_TEST -n -l -e

TEST_NUM=2 # Interface under test
MONI_NUM=1 # Monitoring interface
FREQ=2437 # Frequency for the monitoring interface to monitor



WLAN_TEST=wlan$TEST_NUM
MONI_TEST=moni$TEST_NUM
WLAN_MONI=wlan$MONI_NUM
MONI_MONI=moni$MONI_NUM

MAC_TEST=`ifconfig $WLAN_TEST|grep HWaddr|cut -b 38-55`

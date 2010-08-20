TEST_NUM=1 # Interface under test
MONI_NUM=0 # Monitoring interface
FREQ=2437 # Frequency for the monitoring interface to monitor



WLAN_TEST=wlan$TEST_NUM
MONI_TEST=moni$TEST_NUM
WLAN_MONI=wlan$MONI_NUM
MONI_MONI=moni$MONI_NUM

MAC_TEST=`ifconfig $WLAN_TEST|grep HWaddr|cut -b 38-55`

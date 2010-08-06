#! /bin/bash
IFACE=wlan1
ESSID=blaise-test
FREQ=2437

echo Resetting interface...
ifconfig $IFACE down
sleep 0.5
ifconfig $IFACE up
sleep 0.5

tries=0
successes=0
while true; do
  tries=$(( $tries + 1 ))
  echo -n Connecting...
  MSG=`iw dev $IFACE connect -w $ESSID $FREQ`
  if echo $MSG|grep 'connected to' > /dev/null; then
    successes=$(( successes + 1 ))
    echo -n PASS...
  else
    echo -n FAIL...
    echo $MSG
  fi
  sleep 1 # Without this sleep, does not break rt2800usb
  #sleep .$(( $RANDOM / 33 ))
  echo -n Disconnecting...
  iw dev $IFACE disconnect > /dev/null 2>&1
  if iw dev $IFACE link | grep 'Not connected.' > /dev/null; then
    echo PASS...Successes $successes/$tries
  else
    echo Failed to disconnect! Aborting
    exit 1
  fi
  sleep 0
  #sleep .$(( $RANDOM / 33 ))
done

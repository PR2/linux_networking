#! /bin/sh
if lsmod |grep rt2800usb; then
  rmmod rt2800usb
fi

if lsmdo |grep rt2870sta; then
  rmmod rt2870sta
fi

sleep 1 

case $1 in
  sta)
    MOD=rt2870sta
    DEV=ra0
  ;;
  usb)
    MOD=rt2800usb
    DEV=wlan1
  ;;
  *)
    echo No argument, should be sta or usb.
    exit 1
  ;;

sleep 1
modprobe $MOD
ifconfig $DEV up
echo
iw dev $DEV scan freq 2437
lsmod |grep $MOD

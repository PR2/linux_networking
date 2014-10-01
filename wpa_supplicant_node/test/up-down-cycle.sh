#! /bin/bash
IFACE=$1
FREQ=2437

if [ -z $IFACE ]; then
  echo usage $0 '<interface>'
  exit 1
fi

if [ -n $FREQ ]; then
  FREQ="freq $FREQ"
fi

tries=0
successes=0
DIV=33
while true; do
  tries=$(( $tries + 1 ))
  #T=0.$(( $RANDOM / $DIV )) 
  #echo -n "Cycling: Wait $T, "
  #sleep $T
  echo -n "Down, "
  ifconfig $IFACE down
  #T=0.$(( $RANDOM / $DIV )) 
  #echo -n "Wait $T, "
  #sleep $T
  echo -n "Up, "
  ifconfig $IFACE up
  OUT=`iw dev $IFACE scan $FREQ`
  if [ -z "$OUT" ]; then
    echo "FAIL "
    exit 1
  else
    successes=$(( successes + 1 ))
    echo Pass "(`echo $OUT|wc -c` bytes)" $successes/$tries
  fi
done

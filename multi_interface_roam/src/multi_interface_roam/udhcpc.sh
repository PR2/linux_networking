#! /bin/bash
while true; do
  ifconfig $4 >/dev/null 2>&1 && exec udhcpc "$@"
  sleep 10;
done

#!/usr/bin/env sh

python /Users/anton/dev/ardupilot/Tools/autotest/sim_vehicle.py -v ArduPlane -f plane --add-param-file=crsf-mim.param --out udp:127.0.0.1:14551 --instance 1 --sysid 1 &

sleep 5

python /Users/anton/dev/ardupilot/Tools/autotest/sim_vehicle.py -v ArduPlane -f plane --add-param-file=crsf-mim.param --out udp:127.0.0.1:14552 --instance 2 --sysid 2 &

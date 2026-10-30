#!/bin/bash
# open_bag.sh BAG -- open a bag in a new tab of the running Foxglove on :0
F=~/sim/foxglove; export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus XDG_RUNTIME_DIR=/run/user/1000 LD_LIBRARY_PATH=~/foxglove-portable/usr/lib:$LD_LIBRARY_PATH
nohup ~/foxglove-portable/opt/Foxglove/foxglove-studio "$(readlink -f "$1")" >> $F/foxglove.log 2>&1 &

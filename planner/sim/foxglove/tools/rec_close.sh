#!/bin/bash
# rec_close.sh OUT.mp4 SECONDS [NEXT_BAG] -- record the active tab playing, close it (ctrl+w), optionally open the next bag
cd ~/sim/foxglove
tools/record.sh "$1" "$2"
sleep 1; DISPLAY=:0 python3 tools/xin.py key ctrl+w; sleep 3
[ -n "$3" ] && tools/open_bag.sh "$3"
echo done

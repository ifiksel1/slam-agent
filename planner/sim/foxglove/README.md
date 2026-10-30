# Foxglove library tools (sim)

Copies of the tools behind Shannon's `~/sim/foxglove` library (run there, not on the NUC).

- `foxglove_bag.py`: converts a SITL run to a Foxglove bag; adds `/map_low` (env `MAP_LOW=lo,hi`, default `0.05,1.2`).
- `tools/gen.py` + `tools/flights.py`: build the library `README.md` / `index.html` from the run entries.
- `tools/*.sh`, `tools/x*.py`, `tools/tabs.py`, `tools/menu_open.py`: open bags in Foxglove on Shannon's desktop and screen-record them (`record.sh`, `rec_close.sh`, `rec5k.sh`).
- `layouts/`: saved Foxglove layouts (`closeup_layout.json` is the owner's drive close-up view).

Offline analysis / tooling, so Python (the C++ rule covers robot software).

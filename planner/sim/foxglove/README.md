# Foxglove library tools (sim)

Copies of the tools behind Shannon's `~/sim/foxglove` library (run there, not on the NUC).

- `foxglove_bag.py`: converts a SITL run to a Foxglove bag; adds `/map_low` (env `MAP_LOW=lo,hi`, default `0.05,1.2`).
- `tools/gen.py` + `tools/flights.py`: build the library `README.md` / `index.html` from the run entries.
- `tools/*.sh`, `tools/x*.py`, `tools/tabs.py`, `tools/menu_open.py`: open bags in Foxglove on Shannon's desktop and screen-record them (`record.sh`, `rec_close.sh`, `rec5k.sh`).
- `layouts/`: saved Foxglove layouts (`closeup_layout.json` is the owner's drive close-up view).

Offline analysis / tooling, so Python (the C++ rule covers robot software).
- `layouts/click_and_run_layout.json`: the live click-and-run view (`click_and_run/click_and_run.sh`; connect to
  `ws://<shannon>:8765`). Its 3D panel's publish tool sends `/click_and_run/drive` (point: drive) and
  `/click_and_run/fly` (pose: fly).

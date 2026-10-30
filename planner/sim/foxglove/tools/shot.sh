#!/bin/bash
# shot.sh OUT.png [scale]  -- screenshot of :0, scaled to 1920 wide by default
FF=$(~/sim/venv/bin/python -c "import imageio_ffmpeg;print(imageio_ffmpeg.get_ffmpeg_exe())")
taskset -c 0-3 $FF -y -loglevel error -f x11grab -i :0 -frames:v 1 -vf "scale=${2:-1920}:-1" "$1"

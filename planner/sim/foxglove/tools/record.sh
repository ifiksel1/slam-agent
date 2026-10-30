#!/bin/bash
# record.sh OUT.mp4 SECONDS -- record :0 (scaled to 1920x1080, H.264) and press space to start playback
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
FF=$(~/sim/venv/bin/python -c "import imageio_ffmpeg;print(imageio_ffmpeg.get_ffmpeg_exe())")
python3 ~/sim/foxglove/tools/xin.py move 2600 46
taskset -c 0-3 nice -n 5 $FF -y -loglevel error -f x11grab -framerate 20 -draw_mouse 0 -i :0 -t "$2" \
  -vf "crop=3770:2133:70:27,scale=1920:1080:flags=bicubic" -c:v libx264 -preset veryfast -crf 26 -pix_fmt yuv420p -movflags +faststart "$1" &
P=$!
sleep 1.2
python3 ~/sim/foxglove/tools/xin.py key space
wait $P
echo "recorded $1"

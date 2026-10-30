#!/bin/bash
# rec5k.sh OUT.mp4 SECONDS -- record.sh for the 5120x2880 screen: crop=4980:2801:140:79, else the same settings
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
FF=$(~/sim/venv/bin/python -c "import imageio_ffmpeg;print(imageio_ffmpeg.get_ffmpeg_exe())")
python3 ~/sim/foxglove/tools/xin.py move 3400 170
taskset -c 0-3 nice -n 5 $FF -y -loglevel error -f x11grab -framerate 20 -draw_mouse 0 -i :0 -t "$2" \
  -vf "crop=4980:2801:140:79,scale=1920:1080:flags=bicubic" -c:v libx264 -preset veryfast -crf 26 -pix_fmt yuv420p -movflags +faststart "$1" &
P=$!
sleep 1.2
python3 ~/sim/foxglove/tools/xin.py key space
wait $P
echo "recorded $1"

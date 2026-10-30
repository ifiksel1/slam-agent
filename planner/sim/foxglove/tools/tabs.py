#!/usr/bin/env python3
"""tabs.py -- which Foxglove tab slot is active on :0 (Foxglove maximised on the 3840x2160 screen).
Tabs are 200 px wide from x=290; the active tab's background at y=30 is #29292c, others #15151a.
Prints the 1-based active slot (0 if none found). Exit 0 always."""
import subprocess
import sys

import numpy as np
from PIL import Image
import imageio_ffmpeg

FF = imageio_ffmpeg.get_ffmpeg_exe()
raw = subprocess.run([FF, "-loglevel", "error", "-f", "x11grab", "-video_size", "3840x80", "-i", ":0+0,0",
                      "-frames:v", "1", "-f", "rawvideo", "-pix_fmt", "rgb24", "-"],
                     capture_output=True, env={"DISPLAY": ":0", "XAUTHORITY": "/run/user/1000/gdm/Xauthority"}).stdout
if len(raw) != 3840 * 80 * 3:
    print(0)
    sys.exit(0)
row = np.frombuffer(raw, np.uint8).reshape(80, 3840, 3)[30].astype(int)
act = np.all(np.abs(row - [0x29, 0x29, 0x2C]) <= 3, axis=1)
best = 0
for i in range(17):
    a, b = 290 + 200 * i + 20, 490 + 200 * i - 20
    if b > 3500:
        break
    if act[a:b].mean() > 0.8:
        best = i + 1
print(best)

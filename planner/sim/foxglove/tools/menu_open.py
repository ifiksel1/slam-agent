#!/usr/bin/env python3
"""menu_open.py -- exit 0 if Foxglove's layout menu is open on :0 (its search box at (3550,127) is #434346;
closed, that spot is the update banner, #171203)."""
import subprocess
import sys

import imageio_ffmpeg
import numpy as np

raw = subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-loglevel", "error", "-f", "x11grab", "-video_size", "8x8",
                      "-i", ":0+3546,123", "-frames:v", "1", "-f", "rawvideo", "-pix_fmt", "rgb24", "-"],
                     capture_output=True, env={"DISPLAY": ":0", "XAUTHORITY": "/run/user/1000/gdm/Xauthority"}).stdout
if len(raw) != 8 * 8 * 3:
    sys.exit(2)
px = np.frombuffer(raw, np.uint8).reshape(8, 8, 3)[4, 4].astype(int)
sys.exit(0 if np.all(np.abs(px - [0x43, 0x43, 0x46]) <= 4) else 1)

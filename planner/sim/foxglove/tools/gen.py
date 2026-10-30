#!/usr/bin/env python3
"""Write notes.md per flight, layout.json copies, README.md and index.html for ~/sim/foxglove.
Run on Shannon: ~/sim/venv/bin/python gen.py  (from any directory)."""
import html
import os
import re
import shutil
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from flights import CATS, F, SUPERSEDED  # noqa: E402
from rosbags.rosbag1 import Reader  # noqa: E402
import imageio_ffmpeg  # noqa: E402

ROOT = os.path.expanduser("~/sim/foxglove")
FF = imageio_ffmpeg.get_ffmpeg_exe()
CATLAYOUT = {c[0]: c[3] for c in CATS}


def mmss(s):
    s = int(round(s))
    return f"{s // 60}:{s % 60:02d}"


def video_len(p):
    if not os.path.exists(p):
        return None
    out = subprocess.run([FF, "-i", p], capture_output=True, text=True).stderr
    m = re.search(r"Duration: (\d+):(\d+):([\d.]+)", out)
    return int(m[1]) * 3600 + int(m[2]) * 60 + float(m[3]) if m else None


def bag_len(p):
    with Reader(p) as r:
        return (r.end_time - r.start_time) / 1e9


def size(p):
    return os.path.getsize(p) / 1e6


rows = {}
for f in F:
    d = os.path.join(ROOT, f["cat"], f["name"])
    bag = os.path.join(d, f["name"] + ".bag")
    mp4 = os.path.join(d, f["name"] + ".mp4")
    f["bag_s"] = bag_len(bag)
    f["bag_mb"] = size(bag)
    f["mp4_s"] = video_len(mp4)
    f["mp4_mb"] = size(mp4) if os.path.exists(mp4) else 0
    cu = os.path.join(d, f["name"] + "_closeup.mp4")
    f["cu_s"] = video_len(cu)
    f["cu_mb"] = size(cu) if os.path.exists(cu) else 0
    f["has_log"] = os.path.exists(os.path.join(d, "conversion.log"))
    lay = CATLAYOUT[f["cat"]]
    shutil.copyfile(os.path.join(ROOT, "layouts", lay), os.path.join(d, "layout.json"))
    n = [f"# {f['name']}", "", f["desc"], "",
         f"- **Run:** {f['run']}",
         f"- **Map:** {f['map']}",
         f"- **Mission:** {f['mission']}",
         f"- **Result:** {f['result']}",
         f"- **Length:** bag {mmss(f['bag_s'])}" + (f", video {mmss(f['mp4_s'])}" if f["mp4_s"] else ", no video")
         + (f", close-up video {mmss(f['cu_s'])}" if f["cu_s"] else ""),
         f"- **Layout:** `layout.json` (a copy of `layouts/{lay}`)",
         "", "## What to look at", ""]
    n += [f"- {w}" for w in f["watch"]]
    if f["caveats"]:
        n += ["", "## Caveats", ""] + [f"- {c}" for c in f["caveats"]]
    n += ["", "## Files", "",
          f"- `{f['name']}.bag`: the Foxglove-ready bag (see the library README for how it was made)",
          f"- `{f['name']}.mp4`: Foxglove playing the bag at 1x (1920x1080, 20 fps; playback starts ~1 s in)",
          "- `layout.json`: the Foxglove layout used for the video"]
    if f["cu_s"]:
        n.append(f"- `{f['name']}_closeup.mp4`: close-up, the same playback with the camera following the robot "
                 "from ~7 m (layout `layouts/closeup_layout.json`; 1920x1080, 20 fps)")
    if f["has_log"]:
        n.append("- `conversion.log`: the output of the bag conversion")
    n += ["", "Simulation only. See `../../README.md` for the general caveats.", ""]
    open(os.path.join(d, "notes.md"), "w").write("\n".join(n))

# superseded
s = SUPERSEDED
sd = os.path.join(ROOT, "superseded", s["name"])
s["bag_s"] = bag_len(os.path.join(sd, s["name"] + ".bag"))
s["bag_mb"] = size(os.path.join(sd, s["name"] + ".bag"))
shutil.copyfile(os.path.join(ROOT, "layouts", s["layout"]), os.path.join(sd, "layout.json"))
open(os.path.join(sd, "notes.md"), "w").write("\n".join([
    f"# {s['name']} (SUPERSEDED)", "", s["desc"], "", f"**{s['note']}**", "",
    f"- **Run:** {s['run']}", f"- **Map:** {s['map']}",
    "- **Mission:** `example_canusa_A_inside`: take off to 1.5 m, 5 fly legs out and back at 1.5 m.",
    f"- **Length:** bag {mmss(s['bag_s'])}", f"- **Layout:** `layout.json` (a copy of `layouts/{s['layout']}`)", ""]))
open(os.path.join(ROOT, "superseded", "README.md"), "w").write(
    "# Superseded flights\n\nBags kept for reference that should not be used to judge the system.\n\n"
    f"- `{s['name']}/` (cl8): {s['note']}\n")

# ------------------------------------------------------------------ README.md
INTRO = """# Simulation flight library (Foxglove)

Recorded **simulation** flights of the Revolute QuadRover autonomy stack, ready to watch in Foxglove. Each flight
has its own folder with the bag, the Foxglove layout, a screen recording of Foxglove playing it, and short notes.
**Everything here is simulation. There are no real-robot flights in this folder.**

## The pieces

- **SUPER**: the flight planner (from HKU MARS). From the lidar point cloud it builds safe flight corridors
  (the coloured boxes in the 3D view) and plans smooth trajectories through them, replanning several times a second.
- **Track planner**: the ground planner for the drive legs (the cyan path in the drive-and-fly videos).
- **Planner bridge**: turns SUPER's and the track planner's commands into setpoints for the flight controller (FC)
  through MAVROS in Guided mode; it caps the speed, holds when an input goes stale and switches between flight and drive.
- **Mission executive**: runs a mission file leg by leg (take off, fly, land, drive), checks a landing spot before it
  lands, and commands the FC's mode changes and take-offs. It never arms; in these runs the test harness arms SITL.
- **SITL**: ArduPilot software-in-the-loop, the robot's QuadRover firmware (drone + tank) running on a PC with a
  simulated vehicle. MAVROS talks to it exactly as to the real FC.
- **MARSIM**: a lightweight simulator used for the planner-only runs: the drone follows SUPER's trajectory exactly
  ("perfect tracking"), with no FC in the loop.
- **Rendered lidar and SLAM**: in the closed-loop runs, `render_node.py` ray-casts a merged point-cloud map of a real
  site (from earlier real flights) from SITL's pose to make the live cloud (`/planner/cloud`), and publishes SITL's
  true pose as the SLAM odometry (`/Odometry`). SLAM is perfect unless a fault test breaks it on purpose.

## How to watch a flight

1. Install Foxglove desktop (or use app.foxglove.dev).
2. Import the flight's `layout.json`: the layout menu at the top right -> **Import from file...**. Foxglove switches
   to it; next time pick it from the same menu.
3. Open the bag: **Open local file** (or drag the `.bag` onto the window).
4. Press **space** to play. The 3D view is fixed to the map (`world` frame) and starts zoomed out; scroll to zoom,
   drag to orbit, right-drag to pan.

Or just watch the `.mp4` next to the bag: it is Foxglove playing the whole bag at 1x.
`<flight>_closeup.mp4` is the same playback with the camera following the robot from ~7 m: layout
`layouts/closeup_layout.json` (one layout for every bag; its 3D view follows the `drone` TF frame).
In the closed-loop bags the first ~17 s are SITL starting up before it arms, so not much moves.

## What the 3D view shows

- Coloured points: the site map (`/map`, coloured by height). White points: the live (rendered) lidar.
- The robot: body box and the 0.44 m planning sphere (`/robot/*`), with a clearance label
  (`sphere ... m  body ... m`: distance from the sphere / body to the nearest map point; negative = inside).
- SUPER: safe flight corridors (boxes), exploration (`exp_traj`), backup and committed trajectories, the goal.
- Drive and fly: the track planner path (cyan line), the drive goal (cyan arrow), the fly goal (yellow arrow),
  the landing spot and the landing check's markers.
- Side panels: the mission executive's status (JSON), the landing check's answer (or, in the fault layout, the fault
  events); bottom: height, speed and bridge mode, and bands for the bridge mode, the mission phase and the FC mode.
"""

CAVEATS = """## Caveats (read before drawing conclusions)

- **Simulation only.** SITL simulates the vehicle's physics, not the world: **there are no collisions**. A vehicle
  "through" a box, a pole or a roof just passes through; only the floor is felt ("SIM Hit ground" messages).
- **The lidar is rendered from a merged map** of real flights. The map's floor is smeared and uneven (bumps of
  0.1-0.3 m, a 0.2-0.3 m step near map x ~ -1 at Hensel Phelps, a few spikes), which the landing check and the track
  planner see as obstacles. Real lidar noise, dust and moving objects are absent.
- **SLAM is perfect** in the closed-loop runs (SITL's true pose), except where a fault test breaks it. The real
  FAST-LIO's drift and latency are not simulated.
- **Planner-only runs** have perfect tracking: no FC, no wind, no control error. They show the planner, not the robot.
- **Speeds:** the bridge caps flight at 1.0 m/s in simulation (0.5 m/s on the robot until Gate L4); fault-free
  flights still peak at ~1.5 m/s at goal switches.
- **Modes** show as `CMODE(n)` in `/mavros/state` because MAVROS does not know QuadRover's own modes:
  CMODE(18) = QRAltHold, CMODE(19) = QRLoiter, CMODE(21) = QRLand. `/mavros/statustext/recv` has the names.
- **After DONE** the test harness lands SITL with QRLAND, straight down (not part of the mission).
- Shannon runs other SITL jobs; the videos were recorded at 1x without visible stutter, but the timing of a
  run can be affected by load (SITL watchdog resets are why some fault runs were rerun, e.g. f25r, f17r).
"""

MAKING = """## How the bags were made

The raw run bags are huge (2-4 GB). `~/sim/foxglove/foxglove_bag.py` makes them Foxglove-ready: it adds the map as
`/map` (15 cm grid, republished every 5 s so it is there wherever you seek), thins the live lidar to a 10 cm grid,
drops SUPER's `/fsm_node/fsm/path` (republished many times a second, ~770 MB a run), adds plot topics `/viz/speed`,
`/viz/height` and `/viz/phase` (from the executive's status), adds `/tf` world -> `drone` from the odometry when the
run has no `/tf` (the closed-loop runs; the planner-only runs have their own), and writes LZ4-compressed.
The closed-loop bags converted before that change got the same `/tf` afterwards with `tools/add_tf.py`.

```
# planner-only (MARSIM) run:
~/sim/venv/bin/python ~/sim/foxglove/foxglove_bag.py ~/sim/marsim/runs/<run>/run.bag \\
    ~/sim/marsim/maps/<map>.pcd <out>.bag
# closed-loop (SITL) run: planner-frame odometry and cloud, and the map moved into the planner frame
# with the yaw and start in ~/sim/l2/runs/<run>/map_to_planner.json ({"yaw_deg": -90.7, "start": [x, y, z]}):
~/sim/venv/bin/python ~/sim/foxglove/foxglove_bag.py ~/sim/l2/runs/<run>/closed_loop.bag \\
    ~/sim/marsim/maps/hp_1015_carved.pcd <out>.bag /planner/odom /planner/cloud -90.7 <x> <y> <z>
```

It refuses to overwrite an existing bag. Maps: `~/sim/marsim/maps/hp_1015_carved.pcd`,
`~/sim/marsim/maps/canusa_2034.pcd`, and for Gate L2 `~/sim/l2/planner_sim/course/hp_gate_l2.pcd`.
Where a flight folder has a `conversion.log`, it is that command's output ("tightest" = the smallest clearance
label seen in the run; negative on the ground, where the sphere sits in the floor).
The run directories (`~/sim/l2/runs/<run>/`, `~/sim/marsim/runs/<run>/`) keep the full logs and raw bags.
"""

FOLDERS = """## Folders

```
~/sim/foxglove/
  README.md, index.html   this index (index.html: the same, with the videos playable in a browser)
  planner_only/<flight>/  <flight>.bag  <flight>.mp4  <flight>_closeup.mp4  layout.json  notes.md  [conversion.log]
  closed_loop/<flight>/
  drive_fly/<flight>/
  gate_l2/<flight>/
  faults/<flight>/
  superseded/             runs kept for reference only (see its README.md)
  layouts/                all Foxglove layouts (fault_layout.json: drive-and-fly + fault events, sim-truth height, FC mode)
  tools/                  bag conversion (symlinks to the scripts here), screen-recording helpers
  foxglove_bag.py, key.py, click.py   tools, kept here because other scripts call them by this path
```
"""


def table_md(cat):
    out = ["| Flight | What it shows | Result | Length | Files |", "|---|---|---|---|---|"]
    for f in [x for x in F if x["cat"] == cat]:
        p = f"{cat}/{f['name']}/{f['name']}"
        res = f["result"].split(":")[0]
        m = re.search(r"runs/([\w.]+)", f["run"])
        rid = m[1] if m else f["run"].split(" ")[0]
        out.append(f"| **{f['name']}** ({rid}) | {f['desc']} | {res} | {mmss(f['bag_s'])} | "
                   f"[bag]({p}.bag) · [mp4]({p}.mp4) · " + (f"[close-up]({p}_closeup.mp4) · " if f["cu_s"] else "") +
                   f"[notes]({cat}/{f['name']}/notes.md) · "
                   f"[layout]({cat}/{f['name']}/layout.json) |")
    return "\n".join(out)


md = [INTRO]
for cat, title, blurb, lay in CATS:
    md += [f"## {title}", "", blurb, f"Layout: `layouts/{lay}`.", "", table_md(cat), ""]
md += ["## Superseded", "",
       f"- **{SUPERSEDED['name']}** (cl8): {SUPERSEDED['note']} "
       f"[bag](superseded/{SUPERSEDED['name']}/{SUPERSEDED['name']}.bag) · "
       f"[notes](superseded/{SUPERSEDED['name']}/notes.md)", ""]
md += [CAVEATS, MAKING, FOLDERS]
open(os.path.join(ROOT, "README.md"), "w").write("\n".join(md))

# ------------------------------------------------------------------ index.html


def inline(s):
    s = html.escape(s)
    s = re.sub(r"\*\*(.+?)\*\*", r"<strong>\1</strong>", s)
    return re.sub(r"`(.+?)`", r"<code>\1</code>", s)


def md_block_to_html(block):
    out, inlist, incode = [], False, False
    for line in block.splitlines():
        if line.startswith("```"):
            out.append("</pre>" if incode else "<pre>")
            incode = not incode
            continue
        if incode:
            out.append(html.escape(line))
            continue
        if line.startswith("- ") or re.match(r"^\d+\. ", line):
            if not inlist:
                out.append("<ul>")
                inlist = True
            out.append("<li>" + inline(re.sub(r"^(- |\d+\. )", "", line)) + "</li>")
            continue
        if inlist and line.startswith("  "):
            out[-1] = out[-1][:-5] + " " + inline(line.strip()) + "</li>"
            continue
        if inlist:
            out.append("</ul>")
            inlist = False
        if line.startswith("# "):
            out.append(f"<h1>{inline(line[2:])}</h1>")
        elif line.startswith("## "):
            out.append(f"<h2>{inline(line[3:])}</h2>")
        elif line.strip():
            out.append(f"<p>{inline(line)}</p>")
    if inlist:
        out.append("</ul>")
    return "\n".join(out)


cards = []
for cat, title, blurb, lay in CATS:
    cards.append(f"<h2>{html.escape(title)}</h2><p>{inline(blurb)} Layout: <code>layouts/{lay}</code>.</p>")
    cards.append('<div class="grid">')
    for f in [x for x in F if x["cat"] == cat]:
        p = f"{cat}/{f['name']}/{f['name']}"
        closeup = f'<a href="{p}_closeup.mp4">close-up</a> · ' if f["cu_s"] else ""
        res = f["result"].split(":")[0]
        cls = "pass" if res.startswith("PASS") else ("partial" if res.startswith("PARTIAL") else "fail")
        watch = "".join(f"<li>{inline(w)}</li>" for w in f["watch"] + f["caveats"])
        cards.append(f"""<article class="card">
<video controls preload="none" src="{p}.mp4"></video>
<h3>{html.escape(f['name'])} <span class="badge {cls}">{html.escape(res)}</span></h3>
<p class="desc">{inline(f['desc'])}</p>
<p class="meta">{inline(f['run'])} · bag {mmss(f['bag_s'])} · {f['bag_mb']:.0f} MB</p>
<details><summary>Mission, result, what to look at</summary>
<p><strong>Mission:</strong> {inline(f['mission'])}</p><p><strong>Result:</strong> {inline(f['result'])}</p>
<ul>{watch}</ul></details>
<p class="links"><a href="{p}.bag">bag</a> · <a href="{p}.mp4">mp4</a> · {closeup}<a href="{cat}/{f['name']}/notes.md">notes</a> · <a href="{cat}/{f['name']}/layout.json">layout</a></p>
</article>""")
    cards.append("</div>")
cards.append(f"<h2>Superseded</h2><p><strong>{SUPERSEDED['name']}</strong> (cl8): {inline(SUPERSEDED['note'])} "
             f"<a href=\"superseded/{SUPERSEDED['name']}/{SUPERSEDED['name']}.bag\">bag</a> · "
             f"<a href=\"superseded/{SUPERSEDED['name']}/notes.md\">notes</a></p>")

page = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Simulation Flight Library</title>
<style>
:root {{ --bg:#f7f7f5; --fg:#1d1d1b; --muted:#66655f; --card:#fff; --line:#e2e1dc; --code:#efeee9;
  --pass:#1f7a3a; --fail:#b3261e; --partial:#9a6700; }}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{ --bg:#141413; --fg:#ecebe6; --muted:#a3a29b;
  --card:#1e1e1c; --line:#33332f; --code:#2a2a27; --pass:#5cc47a; --fail:#f2766d; --partial:#e0b341; }} }}
body {{ background:var(--bg); color:var(--fg); font:15px/1.5 system-ui, sans-serif; margin:0; padding:24px 16px; }}
main {{ max-width:1200px; margin:0 auto; }}
h1 {{ font-size:28px; margin:0 0 8px; }} h2 {{ margin-top:36px; border-bottom:1px solid var(--line); padding-bottom:4px; }}
code {{ background:var(--code); padding:1px 4px; border-radius:4px; font-size:13px; }}
pre {{ background:var(--code); padding:12px; overflow-x:auto; border-radius:6px; font-size:13px; }}
.grid {{ display:grid; grid-template-columns:repeat(auto-fill, minmax(340px, 1fr)); gap:16px; }}
.card {{ background:var(--card); border:1px solid var(--line); border-radius:8px; padding:12px; }}
.card video {{ width:100%; border-radius:4px; background:#000; aspect-ratio:16/9; }}
.card h3 {{ font-size:16px; margin:8px 0 4px; word-break:break-word; }}
.desc {{ margin:4px 0; }} .meta, .links {{ color:var(--muted); font-size:13px; margin:4px 0; }}
.badge {{ font-size:12px; padding:1px 6px; border-radius:10px; border:1px solid currentColor; white-space:nowrap; }}
.pass {{ color:var(--pass); }} .fail {{ color:var(--fail); }} .partial {{ color:var(--partial); }}
details {{ font-size:14px; }} a {{ color:inherit; }}
</style></head><body><main>
{md_block_to_html(INTRO)}
{''.join(cards)}
{md_block_to_html(CAVEATS)}
{md_block_to_html(MAKING)}
{md_block_to_html(FOLDERS)}
</main></body></html>
"""
open(os.path.join(ROOT, "index.html"), "w").write(page)

# summary for the report
for f in F:
    print(f"{f['cat']}/{f['name']}: bag {mmss(f['bag_s'])} {f['bag_mb']:.0f} MB, video "
          f"{mmss(f['mp4_s']) if f['mp4_s'] else 'MISSING'} {f['mp4_mb']:.1f} MB, close-up "
          f"{mmss(f['cu_s']) if f['cu_s'] else 'MISSING'} {f['cu_mb']:.1f} MB")
print(f"superseded/{SUPERSEDED['name']}: bag {mmss(SUPERSEDED['bag_s'])} {SUPERSEDED['bag_mb']:.0f} MB")

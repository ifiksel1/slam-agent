#!/usr/bin/env python3
"""The failure tests' facts, one row per run: what was injected, how far the vehicle went after the
fault and what the FC, the executive and the bridge did. Reads each run's fault_result.json
(faults/inject.py) and logs/mav_watch.jsonl (faults/mav_watch.py: the FC's modes as SITL reports them,
also when MAVROS is dead). Plain python3, no ROS.

usage: results.py <run dir> [<run dir> ...]      (e.g. results.py ~/sim/l2/runs/f*)
       results.py --detail <run dir>              the full timelines of one run
"""
import json
import os
import sys


def load(run):
    p = os.path.join(run, "fault_result.json")
    if not os.path.exists(p):
        return None
    r = json.load(open(p))
    r["run"] = os.path.basename(run.rstrip("/"))
    mw = os.path.join(run, "logs", "mav_watch.jsonl")
    r["fc"] = [json.loads(x) for x in open(mw)] if os.path.exists(mw) else []
    fm = os.path.join(run, "logs", "fly_mission.log")
    r["fly_end"] = [x.strip() for x in open(fm) if "mission end" in x or "exited" in x] if os.path.exists(fm) else []
    so = os.path.join(run, "logs", "sitl_stdout.log")
    r["sigalrm"] = os.path.exists(so) and "SIGALRM" in open(so, errors="replace").read()
    return r


def fc_after(r, t0, until=None):
    """the FC's mode changes from SITL (mav_watch), relative to the fault"""
    out = []
    for e in r["fc"]:
        if e["kind"] == "mode" and e["t"] >= t0 - 0.5 and (until is None or e["t"] <= until):
            out.append("%s%s@%+.1f" % (e["mode"], "" if e["armed"] else "(disarmed)", e["t"] - t0))
    return out


def texts_after(r, t0, until=None):
    return ["%+.1f %s" % (e["t"] - t0, e["text"]) for e in r["fc"]
            if e["kind"] == "text" and e["t"] >= t0 and (until is None or e["t"] <= until)]


def row(r):
    if not r.get("fault"):
        return "| %s | %s | not triggered: %s | | | | | |" % (r["case"], r["run"], r.get("events", [{}])[-1])
    t0 = r["fault"]["t"]
    until = t0 + r["end"]["dt"]
    before = next((e for e in reversed(r["fc"]) if e["kind"] == "mode" and e["t"] < t0), None)
    fc = ([before["mode"] + "@before"] if before else []) + [x for x in fc_after(r, t0, until) if not x.endswith("@-0.0")]
    ex = []
    for dt, s in r.get("exec", []):
        if dt < 0:
            continue
        x = s.get("phase", "")
        if s.get("reason"):
            x += " (%s)" % s["reason"]
        ex.append("%s@%+.1f" % (x, dt))
    br = []
    for dt, st, why, en, md in r.get("bridge", []):
        if dt >= -0.5:
            br.append("%s%s@%+.1f" % (st, " (%s)" % why if why else "", dt))
    st = r.get("stop")
    stop = ("%.2f m along, %.2f m from the fault point, in %.1f s" % (st["along_m"], st["dist_m"], st["t_s"])
            if st else "did not stop (< 0.1 m/s for 1 s) in the window")
    end = r["end"]
    endtxt = "%s, %s, z %.2f m, %.2f m from the fault point, exec %s%s" % (
        end["mode"], "armed" if end["armed"] else "disarmed", end["pos"][2], end["dist_from_fault_m"],
        end["exec_phase"], "" if end["exec_alive"] else " (executive process gone)")
    box = r.get("box_min_dist_m")
    if box:
        endtxt += "; nearest the box: %.2f m (FC centre to surface)" % box["centre_to_box"]
    inj = "; ".join(a for a in r["actions"] if a != "none") or "observe only"
    return "| %s | %s | %s; %.2f m/s at the fault (%s) | %s; max excursion %.2f m; z %s | %s | %s | %s | %s |" % (
        r["case"], r["run"], inj, r["fault"]["speed"], r["fired_by"], stop, r["max_excursion_m"] or 0,
        r["z_min_max"], ", ".join(fc), "; ".join(ex[:8]), "; ".join(br[:8]), endtxt)


def detail(r):
    print(json.dumps({k: r.get(k) for k in ("case", "run", "trigger", "actions", "fired_by", "end_why", "fault", "stop",
                                            "max_excursion_m", "along_at_0.5s_m", "z_min_max", "end", "box_min_dist_m",
                                            "mode_before", "planner_md5", "fly_end", "sigalrm")}, indent=1))
    t0 = r["fault"]["t"] if r.get("fault") else 0
    for k in ("modes", "exec", "bridge", "track", "texts"):
        print("--", k)
        for x in r.get(k, []):
            print("  ", x)
    print("-- FC (SITL, mav_watch)")
    for x in fc_after(r, t0) + texts_after(r, t0):
        print("  ", x)


if __name__ == "__main__":
    if sys.argv[1] == "--detail":
        detail(load(sys.argv[2]))
        sys.exit()
    print("| case | run | injected | stopping / excursion | FC modes (SITL) | executive | planner bridge | end state |")
    print("|---|---|---|---|---|---|---|---|")
    for run in sys.argv[1:]:
        r = load(run)
        if r is not None:
            print(row(r) + (" SITL WATCHDOG RESET" if r["sigalrm"] else ""))

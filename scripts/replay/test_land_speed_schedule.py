#!/usr/bin/env python3
"""
test_land_speed_schedule.py - drive fc_scripts/land_speed_schedule.lua in a real Lua interpreter.

Stubs the ArduPilot bindings (param/arming/vehicle/rangefinder/gcs) and flies a LAND from 3 m to the
ground, checking what LAND_SPEED and LAND_SPEED_HIGH the FC would be reading at each altitude, and
that both go back to their saved values the moment the LAND ends.

The rangefinder dropout cases use what the MTF-01 actually reported in FC log 231: status 2
(OutOfRangeLow) with distance 0 at ~5 m, when it lost the ground. Treating that as "near the
ground" would slow a descent from 5 m to 25 cm/s.

Run: pip install lupa && python3 scripts/replay/test_land_speed_schedule.py
"""
import os
import sys

try:
    import lupa
except ImportError:
    sys.exit("needs lupa: pip install lupa")

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "..", "..", "fc_scripts", "land_speed_schedule.lua")
FAILURES = []

HARNESS = r"""
-- ---- stubbed ArduPilot bindings ----
local sent = {}
_params = {}          -- full name -> value, the FC's RAM copy
_eeprom = {}          -- full name -> value, never written by param:set

param = {
  add_table = function(self, k, p, n)
    if _taken and _taken[k] then return false end
    _used_key = k
    return true
  end,
  add_param = function(self, k, i, n, d) _params["LSS_" .. n] = d; return true end,
}
function Parameter(full)
  return {
    get = function(self) return _params[full] end,
    set = function(self, v) _params[full] = v; _sets = _sets + 1; return true end,
  }
end

arming = {is_armed = function(self) return _armed end}
vehicle = {get_mode = function(self) return _mode end}
rangefinder = {
  status_orient = function(self, rot) assert(rot == 25); return _rng_status end,
  distance_cm_orient = function(self, rot) assert(rot == 25); return _rng_cm end,
}
gcs = {send_text = function(self, sev, txt) sent[#sent+1] = txt end}

-- ---- test control surface ----
function H_setup(taken)
  _taken = taken
  _used_key = nil
  _sets = 0
  _params = {LAND_SPEED = 50, LAND_SPEED_HIGH = 0}
  _eeprom = {LAND_SPEED = 50, LAND_SPEED_HIGH = 0}
  _armed, _mode, _rng_status, _rng_cm = false, 5, 4, 9
  sent = {}
  update = nil
  local f = loadfile(_SCRIPT_PATH)
  local fn, interval = f()
  update = fn
  return interval
end
function H_tick() if update then update() end end
function H_state(armed, mode) _armed, _mode = armed, mode end
function H_rng(status, cm) _rng_status, _rng_cm = status, cm end
function H_get(name) return _params[name] end
function H_set(name, v) _params[name] = v end
function H_eeprom(name) return _eeprom[name] end
function H_sets() return _sets end
function H_used_key() return _used_key end
function H_texts() local t = table.concat(sent, " | "); sent = {}; return t end
"""

LOITER, LAND, RTL = 5, 9, 6
GOOD, OOR_LOW, NO_DATA = 4, 2, 1


def check(label, got, want):
    ok = got == want
    print("    %-60s %s" % (label, "PASS" if ok else "FAIL (got %r, want %r)" % (got, want)))
    if not ok:
        FAILURES.append(label)


def main():
    L = lupa.LuaRuntime(unpack_returned_tuples=True)
    L.globals()["_SCRIPT_PATH"] = os.path.abspath(SCRIPT)
    L.execute(HARNESS)
    g = L.globals()

    def at(cm, status=GOOD, ticks=1):
        g.H_rng(status, cm)
        for _ in range(ticks):
            g.H_tick()
        return g.H_get("LAND_SPEED"), g.H_get("LAND_SPEED_HIGH")

    print("=== boot ===")
    interval = g.H_setup(None)
    check("loop interval 50 ms", interval, 50)
    check("claims the first candidate key", g.H_used_key(), 93)
    check("announces the saved values", "saved LAND_SPEED 50 / HIGH 0" in g.H_texts(), True)
    g.H_tick()
    check("disarmed in Loiter: params untouched", (g.H_get("LAND_SPEED"), g.H_get("LAND_SPEED_HIGH")), (50, 0))
    check("  and nothing written", g.H_sets(), 0)

    print("\n=== armed Loiter: not ours to touch ===")
    g.H_state(True, LOITER)
    check("Loiter at 2 m", at(200), (50, 0))
    check("Loiter at 30 cm", at(30), (50, 0))

    print("\n=== LAND from 3 m to the ground ===")
    g.H_setup(None)
    g.H_texts()
    g.H_state(True, LAND)
    check("300 cm: 50 / 50", at(300), (50, 50))
    check("  announced", "LAND at 50 cm/s, 25 below 50 cm" in g.H_texts(), True)
    check("120 cm: still 50 (stock would already be ramping)", at(120), (50, 50))
    check("51 cm: still 50", at(51), (50, 50))
    check("49 cm: 25", at(49), (25, 50))
    check("  announced", "slowing to 25" in g.H_texts(), True)
    check("9 cm (on the ground, GNDCLEAR 10): 25", at(9), (25, 50))
    check("EEPROM never written", (g.H_eeprom("LAND_SPEED"), g.H_eeprom("LAND_SPEED_HIGH")), (50, 0))

    print("\n=== the LAND ends: saved values come straight back ===")
    g.H_state(False, LAND)          # landed and auto-disarmed
    g.H_tick()
    check("disarm restores 50 / 0", (g.H_get("LAND_SPEED"), g.H_get("LAND_SPEED_HIGH")), (50, 0))
    g.H_setup(None)
    g.H_state(True, LAND)
    at(40)
    g.H_state(True, LOITER)         # pilot takes it back mid-descent
    g.H_tick()
    check("mode change out of LAND restores 50 / 0", (g.H_get("LAND_SPEED"), g.H_get("LAND_SPEED_HIGH")), (50, 0))
    g.H_state(True, LAND)           # and LANDs again from up high
    check("a new LAND starts fast again, latch cleared", at(250), (50, 50))

    print("\n=== RTL and other modes are left alone ===")
    g.H_setup(None)
    g.H_state(True, RTL)
    check("RTL at 30 cm: untouched", at(30), (50, 0))

    print("\n=== rangefinder dropouts ===")
    g.H_setup(None)
    g.H_state(True, LAND)
    check("log 231 case: OutOfRangeLow, 0 cm, at height -> stays fast", at(0, OOR_LOW), (50, 50))
    check("NoData -> stays fast (today's behaviour)", at(0, NO_DATA), (50, 50))
    check("ground reacquired at 45 cm -> slow", at(45), (25, 50))
    check("dropout after latching -> stays slow", at(0, NO_DATA), (25, 50))
    check("spurious 400 cm after latching -> stays slow", at(400), (25, 50))

    print("\n=== LSS_ENABLE = 0 is a live kill switch ===")
    g.H_setup(None)
    g.H_state(True, LAND)
    at(30)
    g.H_set("LSS_ENABLE", 0)
    g.H_tick()
    check("disabling mid-LAND restores 50 / 0 at once", (g.H_get("LAND_SPEED"), g.H_get("LAND_SPEED_HIGH")), (50, 0))

    print("\n=== tunable from the GCS ===")
    g.H_setup(None)
    g.H_set("LSS_HI_SPD", 60)
    g.H_set("LSS_LO_SPD", 20)
    g.H_set("LSS_LO_ALT", 80)
    g.H_state(True, LAND)
    check("HI 60 above 80 cm", at(100), (60, 60))
    check("LO 20 below 80 cm", at(70), (20, 60))

    print("\n=== no needless param writes ===")
    g.H_setup(None)
    g.H_state(True, LAND)
    at(300, ticks=200)              # 10 s of descent at 20 Hz
    check("one write (HIGH 0->50; SPEED already 50), not one per tick", g.H_sets(), 1)

    print("\n=== key collision ===")
    g.H_setup(L.table_from({93: True, 117: True}))
    check("skips taken keys", g.H_used_key(), 151)
    g.H_setup(L.table_from({93: True, 117: True, 151: True, 177: True, 197: True}))
    check("all taken: says so", "INACTIVE" in g.H_texts(), True)
    g.H_state(True, LAND)
    g.H_tick()
    check("  and leaves LAND alone", (g.H_get("LAND_SPEED"), g.H_get("LAND_SPEED_HIGH")), (50, 0))

    print()
    if FAILURES:
        print("%d FAILED" % len(FAILURES))
        sys.exit(1)
    print("ALL PASS")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
test_lua_gate.py - drive fc_scripts/slam_latency_gate.lua in a real Lua interpreter.

Stubs the ArduPilot bindings (param/arming/mavlink/gcs/millis) and feeds byte-accurate
mavlink_message_t frames, so the decode offsets and the fail-open behaviour are exercised for
real rather than reviewed by eye.

The frames are padded past the payload with 0xAA. That is the point of the test: MAVLink 2 trims
trailing zero bytes, so a 7-character name arrives with payload_len 15, not 18. Code that reads a
fixed char[10] picks up the padding and the name never matches. Zero-padding would hide it.

Run: pip install lupa && python3 scripts/replay/test_lua_gate.py
"""
import os
import struct
import sys

try:
    import lupa
except ImportError:
    sys.exit("needs lupa: pip install lupa")

HERE = os.path.dirname(os.path.abspath(__file__))
GATE = os.path.join(HERE, "..", "..", "fc_scripts", "slam_latency_gate.lua")

NAMED_VALUE_FLOAT_ID = 251
FAILURES = []


def frame(name, value, time_ms=1234, msgid=NAMED_VALUE_FLOAT_ID):
    """Build the mavlink_message_t C struct the scripting binding hands to Lua.

    MAVPACKED, so no padding: checksum(2) magic len incompat compat seq sysid compid (7)
    msgid(3) -> payload at offset 12, i.e. Lua index 13. Offsets confirmed against
    libraries/AP_Scripting/modules/MAVLink/mavlink_msgs.lua:decode_header.
    """
    payload = struct.pack("<If", time_ms, value) + name.encode().ljust(10, b"\0")
    payload = payload.rstrip(b"\0")                     # MAVLink 2 trailing-zero trim
    hdr = struct.pack("<HBBBBBBB", 0, 0xFD, len(payload), 0, 0, 7, 1, 1)
    hdr += int(msgid).to_bytes(3, "little")
    return (hdr + payload).ljust(300, b"\xAA")          # garbage past the payload, deliberately


HARNESS = r"""
-- ---- stubbed ArduPilot bindings ----
local now_ms = 0
local queue = {}
local sent = {}
local auth = {state = "none", msg = ""}

function millis() return {tofloat = function() return now_ms end} end

-- add_table fails when another script already claimed the key with a different prefix, and
-- that claim persists in EEPROM. _taken lets a test reproduce exactly that.
param = {
  add_table = function(self, k, p, n)
    if _taken and _taken[k] then return false end
    _used_key = k
    return true
  end,
  add_param = function(self, k, i, n, d) _defaults[n] = d; return true end,
}
_defaults = {}
_overrides = {}
function Parameter(full)
  local short = string.gsub(full, "^SLG_", "")
  return {get = function(self)
    if _overrides[short] ~= nil then return _overrides[short] end
    return _defaults[short]
  end}
end

arming = {
  -- NOT "_no_slot and nil or 1": in Lua that idiom can never yield nil, because
  -- (true and nil) is nil and (nil or 1) is 1. It silently handed out a slot.
  get_aux_auth_id = function(self)
    _auth_requested = true          -- merely ASKING poisons the pool when it is full
    if _no_slot then return nil end
    return 1
  end,
  set_aux_auth_failed = function(self, id, m) auth.state = "failed"; auth.msg = m end,
  set_aux_auth_passed = function(self, id) auth.state = "passed"; auth.msg = "" end,
}

mavlink = {
  init = function(self, d, n) end,
  register_rx_msgid = function(self, id) _registered = id end,
  receive_chan = function(self)
    if #queue == 0 then return nil end
    return table.remove(queue, 1), 0, 0
  end,
}

gcs = {send_text = function(self, sev, txt) sent[#sent+1] = txt end}

-- ---- test control surface ----
function H_setup(no_slot, taken)
  _no_slot = no_slot
  _taken = taken
  _used_key = nil
  _auth_requested = false
  now_ms, queue, sent = 0, {}, {}
  auth.state, auth.msg = "none", ""
  update = nil
  local f = loadfile(_GATE_PATH)
  local fn, interval = f()
  update = fn
  return interval
end
function H_push(bytes) queue[#queue+1] = bytes end
function H_advance(ms) now_ms = now_ms + ms end
function H_tick() if update then update() end end
function H_auth() return auth.state, auth.msg end
function H_texts() local t = table.concat(sent, " | "); sent = {}; return t end
function H_registered() return _registered end
function H_used_key() return _used_key end
function H_auth_requested() return _auth_requested end
function H_set(name, v) _overrides[name] = v end
"""


def check(label, got, want):
    ok = got == want
    print("    %-56s %s" % (label, "PASS" if ok else "FAIL (got %r, want %r)" % (got, want)))
    if not ok:
        FAILURES.append(label)


def main():
    L = lupa.LuaRuntime(unpack_returned_tuples=True)
    L.globals()["_GATE_PATH"] = os.path.abspath(GATE)
    L.execute(HARNESS)
    g = L.globals()

    def feed(n, name="SLAMLAT", value=70.0, tick_ms=200):
        for _ in range(n):
            g.H_push(frame(name, value))
            g.H_advance(tick_ms)
            g.H_tick()

    print("=== boot: blocked before any SLAMLAT arrives ===")
    interval = g.H_setup(False, None)
    check("registers msgid 251", g.H_registered(), 251)
    check("loop interval 200 ms", interval, 200)
    check("starts blocked", g.H_auth()[0], "failed")
    check("reason names the wait", g.H_auth()[1], "SLAM latency: waiting")

    print("\n=== healthy 70 ms: releases, but only after RELEASE_N ===")
    g.H_setup(False, None)
    feed(11)
    check("still blocked at 11 samples (<12)", g.H_auth()[0], "failed")
    feed(1)
    check("released at 12 samples (3.0 s at the 4 Hz feed)", g.H_auth()[0], "passed")

    print("\n=== cold start 13600 ms: blocks fast ===")
    g.H_setup(False, None)
    feed(1, value=13600.0)
    check("one bad sample is not enough", g.H_auth()[0], "failed")  # still the boot block
    g.H_texts()
    feed(1, value=13600.0)
    st, msg = g.H_auth()
    check("blocked at BLOCK_N=2 (0.4 s)", st, "failed")
    check("reason carries the number", msg, "SLAM latency 13600ms")

    print("\n=== the 25 Aug value, 669 ms ===")
    g.H_setup(False, None)
    feed(20)                                    # get to released first
    check("released while healthy", g.H_auth()[0], "passed")
    feed(2, value=669.0)
    check("669 ms blocks", g.H_auth()[0], "failed")
    check("and only after draining does it clear", g.H_auth()[0], "failed")
    feed(12, value=70.0)
    check("clears after 3 s of good values", g.H_auth()[0], "passed")

    print("\n=== 9999 unknown sentinel from the companion ===")
    g.H_setup(False, None)
    feed(20)
    feed(2, value=9999.0)
    check("sentinel blocks", g.H_auth()[0], "failed")

    print("\n=== FAIL OPEN on silence (the safety-critical case) ===")
    g.H_setup(False, None)
    feed(20, value=9999.0)                      # firmly blocked
    check("blocked before silence", g.H_auth()[0], "failed")
    g.H_texts()
    for _ in range(20):                         # 4 s of no messages, SLG_TOUT is 3 s
        g.H_advance(200)
        g.H_tick()
    check("gate OPENS when the companion goes silent", g.H_auth()[0], "passed")
    check("and says so", "no SLAMLAT" in g.H_texts(), True)

    print("\n=== boundary and robustness ===")
    g.H_setup(False, None)
    feed(20)
    feed(5, value=150.0)
    check("exactly 150 ms blocks (>= not >)", g.H_auth()[0], "failed")
    g.H_setup(False, None)
    feed(20)
    feed(5, value=149.0)
    check("149 ms does not block", g.H_auth()[0], "passed")

    g.H_setup(False, None)
    feed(20)
    for _ in range(20):                         # a different NAMED_VALUE_FLOAT on the same link
        g.H_push(frame("BATTVOLT", 9999.0))
        g.H_advance(200)
        g.H_tick()
    check("ignores other named values entirely", g.H_auth()[0], "passed")
    check("...and treats them as silence, not as data", "no SLAMLAT" in g.H_texts(), True)

    g.H_setup(False, None)
    feed(20)
    for _ in range(5):                          # a different msgid that happens to be queued
        g.H_push(frame("SLAMLAT", 9999.0, msgid=30))
        g.H_advance(200)
        g.H_tick()
    check("ignores non-251 msgids", g.H_auth()[0], "passed")

    print("\n=== SLG_ENABLE = 0 releases rather than latching ===")
    g.H_setup(False, None)
    feed(20, value=9999.0)
    check("blocked while enabled", g.H_auth()[0], "failed")
    g.H_set("ENABLE", 0)
    g.H_tick()
    check("disabling clears the block", g.H_auth()[0], "passed")
    g.H_set("ENABLE", 1)

    print("\n=== one transient sample must NOT block (5 Hz poll vs 4 Hz feed) ===")
    # On 2026-09-02 a single 212 ms cold-start sample blocked arming, because BLOCK_N counted
    # loop iterations and the loop outruns the feed. The companion's interlock, which needs
    # 0.5 s of sustained badness, never moved - that mismatch was the tell.
    g.H_setup(False, None)
    feed(20)
    check("settled open", g.H_auth()[0], "passed")
    g.H_push(frame("SLAMLAT", 212.0)); g.H_advance(200); g.H_tick()
    g.H_advance(200); g.H_tick()          # poll again, no new message
    g.H_advance(200); g.H_tick()
    check("one sample polled 3x does not block", g.H_auth()[0], "passed")
    g.H_push(frame("SLAMLAT", 212.0)); g.H_advance(200); g.H_tick()
    check("two genuine samples do block", g.H_auth()[0], "failed")
    feed(12)
    check("releases after 12 good samples", g.H_auth()[0], "passed")

    print("\n=== param table key collision - the bug that shipped ===")
    # 82 was taken on the real airframe by one of four other scripts, and the original assert
    # killed the script at load: no params, no messages, arming silently ungated. The FC said
    # "Lua: slam_latency_gate.lua:41: SLG: could not add param table" and nothing else.
    g.H_setup(False, L.table_from({82: True}))
    check("falls back off a taken key", g.H_used_key(), 137)
    check("and still arms its gate", g.H_auth()[0], "failed")
    check("announcing which key it took", "key 137" in g.H_texts(), True)

    g.H_setup(False, L.table_from({82: True, 137: True, 163: True}))
    check("walks past several taken keys", g.H_used_key(), 189)

    g.H_setup(False, L.table_from({82: True, 137: True, 163: True, 189: True, 211: True}))
    check("no free key: says INACTIVE rather than dying", "INACTIVE" in g.H_texts(), True)

    print("\n=== no free aux-auth slot ===")
    # On 2026-09-02 this grounded the aircraft. aux_auth_count_max is 3, three other scripts held
    # all three, and get_aux_auth_id() is not a read-only probe: asking when full sets
    # aux_auth_error, which fails prearm with "Too many auxiliary authorisers" for the WHOLE
    # vehicle. There is no way to test for a free slot without consuming one.
    g.H_setup(True, None)
    check("tells the operator how to recover", "SLG_ENABLE=0" in g.H_texts(), True)

    print("\n=== SLG_ENABLE=0 must not touch the slot pool at all ===")
    g.H_setup(False, None)
    check("enabled: does request a slot", g.H_auth_requested(), True)
    g.H_set("ENABLE", 0)
    g.H_setup(False, None)
    check("disabled: never asks, so cannot ground the vehicle", g.H_auth_requested(), False)
    check("and says why", "SLG_ENABLE=0" in g.H_texts(), True)
    g.H_set("ENABLE", 1)

    print()
    if FAILURES:
        print("%d FAILURE(S): %s" % (len(FAILURES), ", ".join(FAILURES)))
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())

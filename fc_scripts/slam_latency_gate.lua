--[[
slam_latency_gate.lua -- refuse to arm while SLAM position output is stale.

WHY THIS EXISTS
  On 25 Aug 2026 this aircraft hard-landed at 3.7 g under LAND and then lifted off again.
  FAST-LIO's publish latency had grown from 82 ms to 669 ms; ArduPilot is told the vision fix is
  VISO_DELAY_MS (75 ms) old and hard-caps compensation at 250 ms, so the EKF fused stale poses.
  A separate finding from the same logs: FAST-LIO's cold start leaves output up to 13.6 s stale
  while still arriving punctually at 20 Hz, and on log 50 the vehicle armed and took off inside
  that window. Nothing on the FC could see it -- staleness is invisible to every existing check,
  because the messages themselves are on time.

  The companion (drift_monitor.py) measures the latency and sends it here as a NAMED_VALUE_FLOAT
  named SLAMLAT, 4 Hz. This script turns that number into a prearm block.

FAIL-OPEN, DELIBERATELY
  If SLAMLAT stops arriving the gate CLEARS and says so. A stuck gate strands the aircraft; a
  cleared one merely returns you to the behaviour you had before this script existed. Only a
  value that actually says "stale" blocks. The companion cooperates with this by reporting an
  explicit large sentinel (9999) when it cannot measure, rather than falling silent.

REQUIRES
  ARMING_CHECK bit 17 (AUX_AUTH, 131072) set, or this script is silently inert -- exactly what
  had already happened to tank_mode v1.10's prearm block on this airframe.
  SCR_ENABLE 1, and a free aux-auth slot (AP_Arming allows 3; tank_mode claims one).
--]]

-- Table keys are claimed by whichever script gets there first, and the claim PERSISTS in
-- EEPROM (AP_Param::add_table compares a CRC of the prefix against the stored one), so a
-- hardcoded key is a landmine: 82 was already taken here by one of the four other scripts on
-- this airframe, and the script died at load with "could not add param table".
-- Searching a fixed candidate list is stable across boots rather than merely lucky: once SLG_
-- is stored under key K, the CRC check makes every later add_table(K) succeed for us and fail
-- for anyone else, so we land on the same K every time.
local PARAM_TABLE_CANDIDATES = {82, 137, 163, 189, 211}
local PARAM_TABLE_PREFIX = "SLG_"
local RUN_INTERVAL_MS = 200          -- 5 Hz, comfortably faster than the 4 Hz feed

-- Asymmetric hysteresis, in loop iterations at RUN_INTERVAL_MS. Blocking is cheap and reversible,
-- so latch it fast; releasing must survive the tail of a cold-start backlog, so make it slow.
-- Mirrors the companion's own lat_k_arm_lock / lat_k_arm_release (10 / 60 scans at 20 Hz).
-- Counted in DISTINCT SAMPLES, not loop iterations. The loop runs at 5 Hz against a 4 Hz feed,
-- so polls outnumber messages and counting iterations let a SINGLE transient sample block
-- arming - observed on 2026-09-02, when one 212 ms cold-start sample tripped the gate while the
-- companion's own interlock (which requires 0.5 s of sustained badness) never moved.
-- At the nominal 4 Hz feed: 2 samples = 0.5 s, 12 samples = 3.0 s.
local BLOCK_N = 2
local RELEASE_N = 12

local NAMED_VALUE_FLOAT_ID = 251
local VALUE_NAME = "SLAMLAT"

local PARAM_TABLE_KEY
for _, key in ipairs(PARAM_TABLE_CANDIDATES) do
   if param:add_table(key, PARAM_TABLE_PREFIX, 3) then
      PARAM_TABLE_KEY = key
      break
   end
end
if not PARAM_TABLE_KEY then
   -- Deliberately not an assert. A script that dies here is indistinguishable from one that was
   -- never installed, which is how this failure hid the first time: no params, no messages after
   -- boot, and arming silently unaffected. Say so instead, and leave the aircraft flyable.
   gcs:send_text(0, "SLG: no free param table key - latency gate INACTIVE")
   return
end

local add_param_failed = false
local function add_param(name, idx, default)
   if not param:add_param(PARAM_TABLE_KEY, idx, name, default) then
      add_param_failed = true
      return nil
   end
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- SLG_ENABLE: 0 disables the gate entirely (auth is released, not left latched).
-- SLG_MS:     block arming at or above this reported latency, milliseconds. 150 matches the
--             companion's lat_arm_ms and is ~2x the 67-78 ms fleet nominal. It is deliberately
--             well below the 250 ms compensation ceiling: arming is discretionary and delaying
--             it is nearly free, so the arming bar is the conservative one.
-- SLG_TOUT:   seconds of SLAMLAT silence after which the gate fails open.
local SLG_ENABLE = add_param("ENABLE", 1, 1)
local SLG_MS     = add_param("MS", 2, 150)
local SLG_TOUT   = add_param("TOUT", 3, 3)

if add_param_failed then
   gcs:send_text(0, "SLG: could not add params - latency gate INACTIVE")
   return
end

-- Check SLG_ENABLE BEFORE asking for a slot. Requesting one when the pool is full is not a
-- read-only probe: AP_Arming::get_aux_auth_id() sets aux_auth_error, and that fails prearm with
-- "Too many auxiliary authorisers" for the WHOLE aircraft, not just for this script. Asking
-- first and apologising afterwards therefore grounds the vehicle. Ordering it this way makes
-- SLG_ENABLE=0 a genuine kill switch that can be set from the GCS without touching the SD card.
if SLG_ENABLE:get() == 0 then
   gcs:send_text(6, "SLG: SLG_ENABLE=0 - latency gate off")
   return
end

-- aux_auth_count_max is 3 and is a compile-time constant, not a parameter. There is no way to
-- ask whether a slot is free without taking one, and asking when full sets aux_auth_error,
-- which fails prearm for the WHOLE vehicle.
--
-- LANDMINE, Copter 4.6.3: slots are never reclaimed. reset_all_aux_auths() - which master calls
-- immediately before lua->run() - does not exist in 4.6.3, so every MAV_CMD_SCRIPTING restart
-- leaks one slot per claiming script, permanently, until a real reboot. On 2026-09-02 that
-- alone exhausted the pool and grounded the aircraft with "Too many auxiliary authorisers",
-- which looked exactly like other scripts having taken every slot. They had not: tank_mode
-- claims exactly one, correctly, from init(). NEVER use a scripting restart to test an
-- aux-auth script on 4.6.3 - reboot instead. Verified working from a clean boot the same day.
local auth_id = arming:get_aux_auth_id()
if not auth_id then
   gcs:send_text(0, "SLG: no aux auth slot free -- SET SLG_ENABLE=0 AND REBOOT")
   return   -- do not reschedule: without a slot there is nothing this script can do
end

mavlink:init(5, 1)
mavlink:register_rx_msgid(NAMED_VALUE_FLOAT_ID)

local last_value = nil
local last_rx_ms = nil
local fresh_sample = false           -- a new SLAMLAT arrived since the last evaluation
local n_bad, n_ok = 0, 0
local state = nil        -- "blocked" | "open" | "stale-open" | "off", for edge-triggered messages

local function announce(new_state, text)
   if state ~= new_state then
      state = new_state
      gcs:send_text(new_state == "open" and 6 or 4, text)   -- INFO on clear, WARNING otherwise
   end
end

-- Drain the RX buffer, keeping the newest SLAMLAT value.
local function poll_mavlink()
   while true do
      local msg = mavlink:receive_chan()
      if msg == nil then
         return
      end
      -- The binding hands us the mavlink_message_t C struct, not the wire format: payload starts
      -- at byte 13 and msgid is a 3-byte field at byte 10 (see AP_Scripting/modules/MAVLink/
      -- mavlink_msgs.lua, decode_header).
      local payload_len = string.unpack("<B", msg, 4)
      local msgid = string.unpack("<I3", msg, 10)
      if msgid == NAMED_VALUE_FLOAT_ID and payload_len >= 9 then
         -- NAMED_VALUE_FLOAT: uint32 time_boot_ms, float value, char name[10].
         local _, value = string.unpack("<If", msg, 13)
         -- MAVLink 2 trims trailing zero bytes, so a 7-character name arrives with
         -- payload_len 15, not 18. Bound the read by the header's length rather than assuming
         -- 10, or we decode whatever happens to sit past the payload in the struct.
         local name = string.sub(msg, 21, 12 + payload_len)
         name = string.match(name, "^[^%z]*") or ""
         if name == VALUE_NAME then
            last_value = value
            last_rx_ms = millis():tofloat()
            fresh_sample = true
         end
      end
   end
end

function update()
   if SLG_ENABLE:get() == 0 then
      arming:set_aux_auth_passed(auth_id)
      announce("off", "SLG: latency gate disabled")
      return update, RUN_INTERVAL_MS
   end

   poll_mavlink()

   local now_ms = millis():tofloat()
   local threshold = SLG_MS:get()
   local age_s = last_rx_ms and ((now_ms - last_rx_ms) * 0.001) or nil

   if age_s == nil or age_s > SLG_TOUT:get() then
      -- Fail open. Never latch on the companion being absent.
      n_bad, n_ok = 0, 0
      arming:set_aux_auth_passed(auth_id)
      announce("stale-open", "SLG: no SLAMLAT - latency gate open")
      return update, RUN_INTERVAL_MS
   end

   if not fresh_sample then
      -- Nothing new to judge. Hold the current verdict rather than re-counting a value we have
      -- already counted; the fail-open timeout above is what handles a feed that has stopped.
      return update, RUN_INTERVAL_MS
   end
   fresh_sample = false

   if last_value >= threshold then
      n_bad, n_ok = n_bad + 1, 0
      if n_bad >= BLOCK_N then
         arming:set_aux_auth_failed(auth_id, string.format("SLAM latency %.0fms", last_value))
         announce("blocked", string.format("SLG: SLAM latency %.0fms - arming blocked", last_value))
      end
   else
      n_ok, n_bad = n_ok + 1, 0
      if n_ok >= RELEASE_N then
         arming:set_aux_auth_passed(auth_id)
         announce("open", string.format("SLG: SLAM latency %.0fms - OK to arm", last_value))
      end
   end

   return update, RUN_INTERVAL_MS
end

-- The key is worth announcing: it is chosen at runtime, and it is the first thing to check if
-- the SLG_ parameters ever appear to reset themselves.
gcs:send_text(6, string.format("SLG: gate loaded (key %d, auth %d, %.0fms)",
                               PARAM_TABLE_KEY, auth_id, SLG_MS:get()))
-- Start blocked-until-proven-good: on boot we have no SLAMLAT yet, and the first update() will
-- fail it open within SLG_TOUT if the companion is genuinely not there. That ordering matters --
-- the dangerous case is arming during a cold start, which is exactly the first few seconds.
arming:set_aux_auth_failed(auth_id, "SLAM latency: waiting")
return update, RUN_INTERVAL_MS

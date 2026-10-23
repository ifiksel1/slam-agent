--[[
landing_gain_reduction_qr.lua -- softer roll/pitch control in the last centimetres of a landing, for QuadRover.

WHY
  The nuc-04 robot bounces on touchdown (owner 2026-10-23; the same on its Copter): ground contact
  shakes it at 28-79 deg/s roll/pitch and the rate controller, on the robot's strong Copter tune
  (rate P 0.46 / 0.33, Copter default 0.135), drives single motors to ~1940 us for ~0.8 s, which
  lifts it again (FC logs 76-79). Copter's own answer, LAND_R/P_MULT (Q_A_LAND_R/P_MULT here), acts
  only once the landing is declared, 0.9-2.4 s after contact on this robot: too late.

WHAT IT DOES
  When the vehicle descends (EKF > 0.1 m/s down) through LGR_ALT (20 cm) on the downward
  rangefinder while flying, it ramps the gains Copter's landed_gain_reduction() scales, for roll and
  pitch, to LGR_MULT (50%) of their saved values:
    Q_A_RAT_RLL_P, Q_A_RAT_RLL_D, Q_A_RAT_PIT_P, Q_A_RAT_PIT_D, Q_A_ANG_RLL_P, Q_A_ANG_PIT_P
  (Copter scales rate P and D and angle P, not I; yaw is left alone). The ramp takes
  2 x Q_A_INPUT_TC, as Copter's. Any landing: QRLand, QRRTL, Auto VTOL_LAND, Guided land,
  QRSmartRTL and a pilot's landing in QRLoiter/QRAltHold/QRStabilize.
  It ramps them back to the saved values as soon as the landing is declared (vehicle not likely
  flying: Copter's land_complete), or the vehicle climbs back above LGR_ALT + RELEASE_CM, or the
  mode changes, or it disarms.

SAFETY PROPERTIES
  * param:set only -- RAM, never EEPROM. A reboot always restores the saved gains; do not "Write
    Params" from a GCS during a landing. The gains are touched only from the start of a reduction
    until they are back (the values to return to are read when it starts).
  * A takeoff never starts it (it needs a descent through LGR_ALT from above LGR_ALT + RELEASE_CM),
    and driving over a bump does not either (no EKF descent).
  * Rangefinder not Good -> it does not start (an active reduction stays until a release).
  * LGR_ENABLE 0 -> the saved gains come back (live kill switch). A Lua error restores them and
    stops the script.
  * The script commands nothing else: no mode change, no arming.

REQUIRES  SCR_ENABLE 1, RNGFND1_ORIENT 25 (down), QuadRover firmware (Q_A_ params).
--]]

-- Unique across the scripts on the vehicle (README: PARAM_TABLE_KEY collisions)
local PARAM_TABLE_CANDIDATES = {109, 131, 157, 181, 209}
local PARAM_TABLE_PREFIX = "LGR_"
local RUN_INTERVAL_MS = 20           -- 50 Hz: the ramp is 0.3 s
local RELEASE_CM = 15                -- released above LGR_ALT + this (climbing away)
local DESCENT_MS = 0.1               -- EKF descent needed to start, m/s
local ROTATION_PITCH_270 = 25        -- downward
local RNG_GOOD = 4

local PARAM_TABLE_KEY
for _, key in ipairs(PARAM_TABLE_CANDIDATES) do
   if param:add_table(key, PARAM_TABLE_PREFIX, 3) then
      PARAM_TABLE_KEY = key
      break
   end
end
if not PARAM_TABLE_KEY then
   gcs:send_text(0, "LGR: no free param table key - INACTIVE")
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

-- LGR_ENABLE:  0 = off; the gains stay at their saved values.
-- LGR_ALT:     rangefinder distance below which the gains are reduced, cm.
-- LGR_MULT:    the reduced gains as a fraction of the saved ones (Copter's LAND_R/P_MULT).
local LGR_ENABLE = add_param("ENABLE", 1, 1)
local LGR_ALT = add_param("ALT", 2, 20)
local LGR_MULT = add_param("MULT", 3, 0.5)

if add_param_failed then
   gcs:send_text(0, "LGR: could not add params - INACTIVE")
   return
end

local GAIN_NAMES = { "Q_A_RAT_RLL_P", "Q_A_RAT_RLL_D", "Q_A_RAT_PIT_P",
                     "Q_A_RAT_PIT_D", "Q_A_ANG_RLL_P", "Q_A_ANG_PIT_P" }
local gains = {}
for i, name in ipairs(GAIN_NAMES) do
   local p = Parameter(name)
   gains[i] = { p = p, saved = p:get() }
end

-- the values to come back to, taken when a reduction starts from full gains (a GCS edit
-- between landings is kept)
local function save_gains()
   for _, g in ipairs(gains) do
      g.saved = g.p:get()
   end
end
local Q_A_INPUT_TC = Parameter("Q_A_INPUT_TC")

local active = false      -- reduction wanted
local ratio = 0.0         -- 0 = saved gains, 1 = fully reduced (Copter's _landed_gain_ratio)
local was_high = false    -- read above LGR_ALT + RELEASE_CM since the last reduction
local active_mode = nil

local function apply(mult)
   for _, g in ipairs(gains) do
      local value = g.saved * mult
      if g.p:get() ~= value then
         g.p:set(value)
      end
   end
end

local function restore_now()
   apply(1.0)
   ratio = 0.0
end

local function release(why)
   if active then
      active = false
      gcs:send_text(6, "LGR: gains back (" .. why .. ")")
   end
end

function update()
   local dt_s = RUN_INTERVAL_MS * 0.001
   if LGR_ENABLE:get() == 0 or not arming:is_armed() then
      release(LGR_ENABLE:get() == 0 and "off" or "disarmed")
      if ratio > 0 then
         restore_now()
      end
      was_high = false
      return
   end

   local mode = vehicle:get_mode()
   local flying = vehicle:get_likely_flying()
   local alt_cm = LGR_ALT:get()
   local dist_cm = nil
   if rangefinder:status_orient(ROTATION_PITCH_270) == RNG_GOOD then
      dist_cm = rangefinder:distance_orient(ROTATION_PITCH_270) * 100
   end

   if active then
      if not flying then
         release("landed")
      elseif mode ~= active_mode then
         release("mode")
      elseif dist_cm ~= nil and dist_cm > alt_cm + RELEASE_CM then
         release("climbing")
      end
   end

   if dist_cm ~= nil and dist_cm > alt_cm + RELEASE_CM then
      was_high = true
   end
   if not active and flying and was_high and dist_cm ~= nil and dist_cm < alt_cm then
      local vel = ahrs:get_velocity_NED()
      if vel ~= nil and vel:z() > DESCENT_MS then
         if ratio == 0 then
            save_gains()
         end
         active = true
         active_mode = mode
         was_high = false
         gcs:send_text(6, string.format("LGR: %.0f cm, roll/pitch gains %.0f%%", dist_cm, LGR_MULT:get() * 100))
      end
   end

   -- Copter's ramp: full scale change in 2 x input TC
   local tc = Q_A_INPUT_TC:get()
   local step = 1.0
   if tc > 0 then
      step = dt_s / (2.0 * tc)
   end
   -- the gains are only touched while a reduction is on or ramping back
   if active or ratio > 0 then
      if active then
         ratio = math.min(1.0, ratio + step)
      else
         ratio = math.max(0.0, ratio - step)
      end
      apply(1.0 - ratio + LGR_MULT:get() * ratio)
   end
end

-- A Lua error stops the script. Without this the RAM gains would stay reduced until reboot; instead
-- put the saved gains back, say why, and stop.
local function protected_update()
   local ok, err = pcall(update)
   if not ok then
      pcall(restore_now)
      gcs:send_text(0, "LGR: error, saved gains restored, stopped: " .. tostring(err))
      return
   end
   return protected_update, RUN_INTERVAL_MS
end

-- one STATUSTEXT holds 50 characters
gcs:send_text(6, string.format("LGR: loaded (key %d, %.0f%% below %.0f cm)",
                               PARAM_TABLE_KEY, LGR_MULT:get() * 100, LGR_ALT:get()))
return protected_update, RUN_INTERVAL_MS

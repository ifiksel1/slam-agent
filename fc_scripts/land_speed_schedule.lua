--[[
land_speed_schedule.lua -- two-stage LAND descent on the downward rangefinder.

WHAT IT DOES
  In LAND, while armed: descend at LSS_HI_SPD (50 cm/s) until the rangefinder reads below LSS_LO_ALT
  (50 cm), then at LSS_LO_SPD (25 cm/s) to touchdown. Outside LAND it puts LAND_SPEED and
  LAND_SPEED_HIGH back to the values they had at boot.

WHY A SCRIPT
  Stock Copter 4.6.3 cannot step at 0.5 m. Mode::land_run_vertical_control (ArduCopter/mode.cpp:647,660)
  uses MAX(LAND_ALT_LOW, 100), so the slow stage never starts below 1 m, and the change is a
  sqrt_controller ramp rather than a step. What it does honour exactly is the clamp
      cmb_rate = constrain(cmb_rate, -LAND_SPEED_HIGH, -LAND_SPEED)
  and both params are read every loop. Holding LAND_SPEED_HIGH = LAND_SPEED = HI pins the descent
  at HI whatever the ramp asks for; dropping LAND_SPEED to LO below the threshold pins it at LO
  (below 1 m the ramp demands a climb, so the clamp's upper bound, -LAND_SPEED, is what wins).

SAFETY PROPERTIES
  * param:set only -- RAM, never EEPROM. A reboot always restores the saved values, and a GCS
    parameter refresh during a LAND shows the temporary values, so do not "Write Params" mid-LAND.
  * Rangefinder not Good -> HI, i.e. today's behaviour (the saved LAND_SPEED is 50). The MTF-01
    reports OutOfRangeLow with distance 0 when it loses the ground at height (seen at ~5 m in FC log
    231), so status 2 is NOT treated as "close to the ground".
  * Once LO is latched in a LAND it stays latched until the LAND ends: being under 0.5 m is not
    something a sensor dropout should undo.
  * The script commands nothing: no mode change, no arming. It only shapes a descent the FC is
    already flying.

REQUIRES  SCR_ENABLE 1, RNGFND1_ORIENT 25 (down).
--]]

-- Table keys must be unique across every script on the vehicle, and add_table only accepts 0-200.
-- Same candidate-search scheme as slam_latency_gate.lua; none of these overlap its list or TANK_ (73).
local PARAM_TABLE_CANDIDATES = {93, 117, 151, 177, 197}
local PARAM_TABLE_PREFIX = "LSS_"
local RUN_INTERVAL_MS = 50           -- 20 Hz; at 0.5 m/s that is 2.5 cm between looks

local MODE_LAND = 9
local ROTATION_PITCH_270 = 25        -- downward
local RNG_GOOD = 4

local PARAM_TABLE_KEY
for _, key in ipairs(PARAM_TABLE_CANDIDATES) do
   if param:add_table(key, PARAM_TABLE_PREFIX, 4) then
      PARAM_TABLE_KEY = key
      break
   end
end
if not PARAM_TABLE_KEY then
   gcs:send_text(0, "LSS: no free param table key - land speed schedule INACTIVE")
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

-- LSS_ENABLE:  0 = off; LAND_SPEED / LAND_SPEED_HIGH left at their saved values.
-- LSS_HI_SPD:  descent above LSS_LO_ALT, cm/s.
-- LSS_LO_SPD:  descent below LSS_LO_ALT, cm/s. Below LAND_SPEED's documented 30 minimum; the
--              firmware does not enforce that range, Mission Planner only warns.
-- LSS_LO_ALT:  rangefinder distance at which to slow down, cm.
local LSS_ENABLE = add_param("ENABLE", 1, 1)
local LSS_HI_SPD = add_param("HI_SPD", 2, 50)
local LSS_LO_SPD = add_param("LO_SPD", 3, 25)
local LSS_LO_ALT = add_param("LO_ALT", 4, 50)

if add_param_failed then
   gcs:send_text(0, "LSS: could not add params - land speed schedule INACTIVE")
   return
end

local LAND_SPEED = Parameter("LAND_SPEED")
local LAND_SPEED_HIGH = Parameter("LAND_SPEED_HIGH")

-- The saved values, captured before this script has touched anything.
local saved_speed = LAND_SPEED:get()
local saved_speed_high = LAND_SPEED_HIGH:get()

local active = false      -- we currently own LAND_SPEED / LAND_SPEED_HIGH
local low_latched = false -- went below LSS_LO_ALT during this LAND

local function set_if_changed(p, value)
   if p:get() ~= value then
      p:set(value)
   end
end

local function restore()
   if active then
      set_if_changed(LAND_SPEED, saved_speed)
      set_if_changed(LAND_SPEED_HIGH, saved_speed_high)
      active = false
   end
   low_latched = false
end

function update()
   if LSS_ENABLE:get() == 0 or not arming:is_armed() or vehicle:get_mode() ~= MODE_LAND then
      restore()
      return update, RUN_INTERVAL_MS
   end

   local hi = LSS_HI_SPD:get()
   local lo = LSS_LO_SPD:get()

   if not low_latched and rangefinder:status_orient(ROTATION_PITCH_270) == RNG_GOOD then
      local dist_cm = rangefinder:distance_cm_orient(ROTATION_PITCH_270)
      if dist_cm < LSS_LO_ALT:get() then
         low_latched = true
         gcs:send_text(6, string.format("LSS: %d cm, slowing to %.0f cm/s", dist_cm, lo))
      end
   end

   if not active then
      active = true
      gcs:send_text(6, string.format("LSS: LAND at %.0f cm/s, %.0f below %.0f cm",
                                     hi, lo, LSS_LO_ALT:get()))
   end
   set_if_changed(LAND_SPEED_HIGH, hi)
   set_if_changed(LAND_SPEED, low_latched and lo or hi)

   return update, RUN_INTERVAL_MS
end

gcs:send_text(6, string.format("LSS: loaded (key %d, saved LAND_SPEED %.0f / HIGH %.0f)",
                               PARAM_TABLE_KEY, saved_speed, saved_speed_high))
return update, RUN_INTERVAL_MS

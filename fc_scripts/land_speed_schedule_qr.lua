--[[
land_speed_schedule_qr.lua -- two-stage QRLand descent on the downward rangefinder, for QuadRover.

QuadRover port of land_speed_schedule.lua (Copter). Same behaviour, same LSS_ parameters
(cm/s, cm): install ONE of the two, never both (they share the LSS_ table).

WHAT IT DOES
  In QRLand (mode 21), while armed: descend at LSS_HI_SPD (50 cm/s) until the rangefinder reads
  below LSS_LO_ALT (50 cm), then at LSS_LO_SPD (25 cm/s) to touchdown. Outside QRLand it puts
  Q_LAND_SPD and Q_LAND_ALT back to the values they had at boot.

HOW IT DRIVES QRLAND
  QuadRover::landing_descent_rate_ms() reads both params every loop:
      height < Q_LAND_ALT  ->  Q_LAND_SPD                (m/s)
      otherwise            ->  MAX(Q_LAND_SPD, 2.0 m/s)  (upper stage is hard-coded)
  So the script holds Q_LAND_ALT far above any landing height (QR_HOLD_ALT_M) to keep the whole
  descent on Q_LAND_SPD, and steps Q_LAND_SPD from HI to LO below LSS_LO_ALT.
  Note the units: LSS_ params are cm/s and cm like the Copter script; Q_LAND_SPD is m/s.

SAFETY PROPERTIES (unchanged from the Copter script)
  * param:set only -- RAM, never EEPROM. A reboot always restores the saved values; do not
    "Write Params" from a GCS mid-QRLand.
  * Rangefinder not Good -> HI. The MTF-01 reports OutOfRangeLow with distance 0 when it loses
    the ground at height (FC log 231), so status 2 is NOT treated as "close to the ground".
  * Once LO is latched in a QRLand it stays latched until the QRLand ends.
  * The script commands nothing: no mode change, no arming. On touchdown QuadRover itself switches
    to Manual, which ends QRLand, and the script restores both params.
  * QRRTL and Auto's VTOL land share landing_descent_rate_ms() but are NOT shaped: the script only
    acts in mode 21.

REQUIRES  SCR_ENABLE 1, RNGFND1_ORIENT 25 (down), QuadRover firmware (Q_LAND_SPD / Q_LAND_ALT).
--]]

-- Same candidate keys and prefix as land_speed_schedule.lua: the two are alternatives.
local PARAM_TABLE_CANDIDATES = {93, 117, 151, 177, 197}
local PARAM_TABLE_PREFIX = "LSS_"
local RUN_INTERVAL_MS = 50           -- 20 Hz; at 0.5 m/s that is 2.5 cm between looks

local MODE_QRLAND = 21
local ROTATION_PITCH_270 = 25        -- downward
local RNG_GOOD = 4
local QR_HOLD_ALT_M = 1000           -- Q_LAND_ALT while active: whole descent on Q_LAND_SPD

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

-- LSS_ENABLE:  0 = off; Q_LAND_SPD / Q_LAND_ALT left at their saved values.
-- LSS_HI_SPD:  descent above LSS_LO_ALT, cm/s.
-- LSS_LO_SPD:  descent below LSS_LO_ALT, cm/s.
-- LSS_LO_ALT:  rangefinder distance at which to slow down, cm.
local LSS_ENABLE = add_param("ENABLE", 1, 1)
local LSS_HI_SPD = add_param("HI_SPD", 2, 50)
local LSS_LO_SPD = add_param("LO_SPD", 3, 25)
local LSS_LO_ALT = add_param("LO_ALT", 4, 50)

if add_param_failed then
   gcs:send_text(0, "LSS: could not add params - land speed schedule INACTIVE")
   return
end

local Q_LAND_SPD = Parameter("Q_LAND_SPD")
local Q_LAND_ALT = Parameter("Q_LAND_ALT")

-- The saved values, captured before this script has touched anything.
local saved_spd = Q_LAND_SPD:get()
local saved_alt = Q_LAND_ALT:get()

local active = false      -- we currently own Q_LAND_SPD / Q_LAND_ALT
local low_latched = false -- went below LSS_LO_ALT during this QRLand

local function set_if_changed(p, value)
   if p:get() ~= value then
      p:set(value)
   end
end

local function restore()
   if active then
      set_if_changed(Q_LAND_SPD, saved_spd)
      set_if_changed(Q_LAND_ALT, saved_alt)
      active = false
   end
   low_latched = false
end

function update()
   if LSS_ENABLE:get() == 0 or not arming:is_armed() or vehicle:get_mode() ~= MODE_QRLAND then
      restore()
      return update, RUN_INTERVAL_MS
   end

   local hi = LSS_HI_SPD:get()
   local lo = LSS_LO_SPD:get()

   if not low_latched and rangefinder:status_orient(ROTATION_PITCH_270) == RNG_GOOD then
      -- 4.7 returns metres as a float; round so the %d below gets an integer
      -- (string.format("%d", 40.3) is a Lua 5.3 error that kills the script)
      local dist_cm = math.floor(rangefinder:distance_orient(ROTATION_PITCH_270) * 100 + 0.5)
      if dist_cm < LSS_LO_ALT:get() then
         low_latched = true
         gcs:send_text(6, string.format("LSS: %d cm, slowing to %.0f cm/s", dist_cm, lo))
      end
   end

   if not active then
      active = true
      gcs:send_text(6, string.format("LSS: QRLand at %.0f cm/s, %.0f below %.0f cm",
                                     hi, lo, LSS_LO_ALT:get()))
   end
   set_if_changed(Q_LAND_ALT, QR_HOLD_ALT_M)
   set_if_changed(Q_LAND_SPD, (low_latched and lo or hi) * 0.01)

   return update, RUN_INTERVAL_MS
end

-- A Lua error stops the script. Without this the RAM overrides (Q_LAND_ALT 1000) would stay until
-- reboot; instead put the saved values back, say why, and stop.
local function protected_update()
   local ok, err = pcall(update)
   if not ok then
      pcall(function()
         set_if_changed(Q_LAND_SPD, saved_spd)
         set_if_changed(Q_LAND_ALT, saved_alt)
      end)
      gcs:send_text(0, "LSS: error, saved Q_LAND_ params restored, stopped: " .. tostring(err))
      return
   end
   return protected_update, RUN_INTERVAL_MS
end

gcs:send_text(6, string.format("LSS: QR loaded (key %d, saved Q_LAND_SPD %.2f m/s, Q_LAND_ALT %.1f m)",
                               PARAM_TABLE_KEY, saved_spd, saved_alt))
return protected_update, RUN_INTERVAL_MS

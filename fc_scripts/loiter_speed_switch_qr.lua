--[[
loiter_speed_switch_qr.lua -- QRLoiter speed (and lean) from a 3-position switch on the radio, for QuadRover.

WHAT IT DOES
  Reads the switch on the RC channel whose RCx_OPTION is QLS_RC_OPT (default 300, "Scripting1")
  and sets QRLoiter's maximum horizontal speed Q_LT_SPEED_MS and its pilot lean Q_LT_ANG_MAX
  (AC_Loiter, Copter's LOIT_SPEED_MS / LOIT_ANG_MAX):
    low  -> QLS_SPD_LO   0.5 m/s, QLS_ANG_LO   5 deg
    mid  -> QLS_SPD_MID  1.5 m/s, QLS_ANG_MID 10 deg
    high -> QLS_SPD_HI   4.0 m/s, QLS_ANG_HI  20 deg
  Why both: in AC_Loiter full stick asks for g*tan(Q_LT_ANG_MAX) and a drag term
  (accel * v / Q_LT_SPEED_MS) balances it at the speed limit, so the speed approaches the limit
  with a time constant speed / (g*tan(lean)). At 5 deg that is 0.6 s for 0.5 m/s but 4.7 s for
  4 m/s. The pairs keep it 0.6-1.1 s: 0.5 m/s / 5 deg is the cage setting (log 60), 4 m/s / 20 deg
  the free-flight set (quadrover_from_pixracer.param). Q_A_ANGLE_MAX still caps the lean.
  AC_Loiter reads both every loop (calc_desired_velocity), so a flip takes effect at once, also in
  the air. Slowing down in flight is flown as Copter's loiter does: the target velocity is clamped
  and the vehicle brakes.
  Nothing else changes speed: only AC_Loiter (QRLoiter, and QRRTL's climb, which has no pilot
  input) reads Q_LT_SPEED_MS; Auto, Guided and the returns fly at Q_WP_SPD.
  While enabled the switch owns both: a value set from a GCS is put back to the switch's.

SETUP
  SCR_ENABLE 1; RCx_OPTION 300 on the switch's channel (x = the channel; nothing else may use 300).

SAFETY PROPERTIES
  * param:set only -- RAM, never EEPROM. A reboot restores the saved Q_LT_SPEED_MS and
    Q_LT_ANG_MAX until the script reads the switch; do not "Write Params" from a GCS after a flip
    unless meant.
  * No switch channel (no RCx_OPTION = QLS_RC_OPT) -> says so once, changes nothing.
  * No valid RC input (receiver lost) -> keeps the current speed.
  * QLS_ENABLE 0 -> puts the saved values back and stops changing them (live kill switch).
  * A Lua error puts the saved values back and stops the script.
  * The script commands nothing else: no mode change, no arming.

REQUIRES  SCR_ENABLE 1, QuadRover firmware (Q_LT_SPEED_MS, Q_LT_ANG_MAX).
--]]

-- Unique across the scripts on the vehicle (README: PARAM_TABLE_KEY collisions)
local PARAM_TABLE_CANDIDATES = {101, 127, 143, 171, 203}
local PARAM_TABLE_PREFIX = "QLS_"
local RUN_INTERVAL_MS = 100          -- 10 Hz

local PARAM_TABLE_KEY
for _, key in ipairs(PARAM_TABLE_CANDIDATES) do
   if param:add_table(key, PARAM_TABLE_PREFIX, 8) then
      PARAM_TABLE_KEY = key
      break
   end
end
if not PARAM_TABLE_KEY then
   gcs:send_text(0, "QLS: no free param table key - loiter speed switch INACTIVE")
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

-- QLS_ENABLE:  0 = off; Q_LT_SPEED_MS back at its saved value.
-- QLS_RC_OPT:  the RCx_OPTION that marks the switch channel (300-307 = Scripting1-8).
-- QLS_SPD_LO / QLS_SPD_MID / QLS_SPD_HI:  QRLoiter speed for each switch position, m/s.
-- QLS_ANG_LO / QLS_ANG_MID / QLS_ANG_HI:  QRLoiter pilot lean for each position, deg.
local QLS_ENABLE  = add_param("ENABLE", 1, 1)
local QLS_RC_OPT  = add_param("RC_OPT", 2, 300)
local QLS_SPD_LO  = add_param("SPD_LO", 3, 0.5)
local QLS_SPD_MID = add_param("SPD_MID", 4, 1.5)
local QLS_SPD_HI  = add_param("SPD_HI", 5, 4.0)
local QLS_ANG_LO  = add_param("ANG_LO", 6, 5)
local QLS_ANG_MID = add_param("ANG_MID", 7, 10)
local QLS_ANG_HI  = add_param("ANG_HI", 8, 20)

if add_param_failed then
   gcs:send_text(0, "QLS: could not add params - loiter speed switch INACTIVE")
   return
end

local Q_LT_SPEED_MS = Parameter("Q_LT_SPEED_MS")
local Q_LT_ANG_MAX = Parameter("Q_LT_ANG_MAX")

-- The saved values, captured before this script has touched anything.
local saved_spd = Q_LT_SPEED_MS:get()
local saved_ang = Q_LT_ANG_MAX:get()

local POS_NAMES = { [0] = "low", [1] = "mid", [2] = "high" }
local SPEEDS = { [0] = QLS_SPD_LO, [1] = QLS_SPD_MID, [2] = QLS_SPD_HI }
local ANGLES = { [0] = QLS_ANG_LO, [1] = QLS_ANG_MID, [2] = QLS_ANG_HI }

local last_pos = nil      -- switch position last applied
local active = false      -- we currently own Q_LT_SPEED_MS
local warned_no_channel = false

local function restore()
   if active then
      Q_LT_SPEED_MS:set(saved_spd)
      Q_LT_ANG_MAX:set(saved_ang)
      active = false
      last_pos = nil
      gcs:send_text(6, string.format("QLS: off, QRLoiter %.1f m/s, %.0f deg", saved_spd, saved_ang))
   end
end

function update()
   if QLS_ENABLE:get() == 0 then
      restore()
      return
   end

   local chan = rc:find_channel_for_option(math.floor(QLS_RC_OPT:get()))
   if chan == nil then
      if not warned_no_channel then
         warned_no_channel = true
         gcs:send_text(4, string.format("QLS: no RC channel with option %d", math.floor(QLS_RC_OPT:get())))
      end
      return
   end
   warned_no_channel = false

   if not rc:has_valid_input() then
      return
   end

   local pos = chan:get_aux_switch_pos()
   local spd = SPEEDS[pos]:get()
   local ang = ANGLES[pos]:get()
   if pos ~= last_pos or Q_LT_SPEED_MS:get() ~= spd or Q_LT_ANG_MAX:get() ~= ang then
      Q_LT_SPEED_MS:set(spd)
      Q_LT_ANG_MAX:set(ang)
      active = true
      -- a parameter edit while the switch is still is not news
      if pos ~= last_pos then
         gcs:send_text(6, string.format("QLS: QRLoiter speed %.1f m/s, lean %.0f deg (%s)", spd, ang, POS_NAMES[pos]))
      end
      last_pos = pos
   end
end

-- A Lua error stops the script. Without this the RAM overrides would stay until reboot; instead
-- put the saved values back, say why, and stop.
local function protected_update()
   local ok, err = pcall(update)
   if not ok then
      pcall(function()
         Q_LT_SPEED_MS:set(saved_spd)
         Q_LT_ANG_MAX:set(saved_ang)
      end)
      gcs:send_text(0, "QLS: error, saved Q_LT_ params restored, stopped: " .. tostring(err))
      return
   end
   return protected_update, RUN_INTERVAL_MS
end

-- one STATUSTEXT holds 50 characters
gcs:send_text(6, string.format("QLS: loaded (key %d, saved %.1f m/s %.0f deg)",
                               PARAM_TABLE_KEY, saved_spd, saved_ang))
return protected_update, RUN_INTERVAL_MS

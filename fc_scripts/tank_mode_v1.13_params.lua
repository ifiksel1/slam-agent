-- =========================================
--  Split-Arcade Tank Script  (Pixhawk Lua)
--  Right stick = throttle, Left stick = steer
--  v1.10 all tunables exposed as TANK_* parameters (visible in Mission Planner)
--        + nav source selected via native EKF3 source sets (no param writes)
--        + per-track direction reverse (SERVOn_REVERSED does not affect RCIN
--          passthrough outputs, so track polarity must be handled here)
--        + real prearm block while in tank mode, and a spoken refusal when
--          the tank switch is flipped with the vehicle armed
--        + steering reverse, for when rotation is mirrored but forward and
--          reverse are already correct
--  v1.11 flight speed select: the tank speed switch (TANK_SPD_CH) now also
--        picks between two flying speeds in drone mode, by scaling the pilot's
--        roll and pitch stick input. Roll is routed through the script for
--        this, the way pitch and yaw already were.
--        Stick centres and end points are read from AP's own RCn_TRIM/MIN/MAX
--        rather than duplicated into TANK_ params, so the radio calibration
--        stays the single source of truth.
--        Deliberately does NOT touch LOIT_SPEED or the PSC gains. Loiter's
--        drag term is  accel_max * v / LOIT_SPEED, so its time constant is
--        LOIT_SPEED / accel_max. Scaling the stick moves the equilibrium
--        speed without moving that time constant, so the position controller
--        tune stays valid. Lowering LOIT_SPEED instead would shorten the time
--        constant and force a PSC_VELXY retune.
--  v1.12 GCS messages say WHICH domain they are about. One switch now selects
--        two different slow/fast pairs -- track speed in tank mode, stick
--        scale in flight -- so "SPLIT_ARCADE_MODE: flight speed SLOW" named
--        the script, which the pilot already knows, and not the thing that
--        actually changed. Routine status messages are now prefixed TANK or
--        FLIGHT. The script name is kept on the boot banner and on warnings,
--        where the point IS to identify which script is speaking.
--        The speed select also moved out of the drone-only branch: it is one
--        switch with one hysteresis, read once per loop, announced in both
--        modes and re-announced on a mode change, because the same switch
--        position means a different thing either side of that change.
--  v1.13 flight speed is set in cm/s, not as a stick fraction. The script
--        inverts Loiter's own steady-state relation to get the stick
--        fraction, then converts that to a scale using the live radio
--        calibration. Everything the old hand-derived 0.293 silently depended
--        on -- LOIT_SPEED, LOIT_ANG_MAX, and the output channel's DZ and MAX
--        -- is now read at runtime, so changing any of them no longer
--        invalidates the setting. Fast mode has no speed parameter: it is
--        full stick, which is LOIT_SPEED, the same number the slow ratio is
--        computed against. The hysteresis band is a parameter too.
-- =========================================
local SCRIPT_NAME = "SPLIT_ARCADE_MODE"
local VERSION     = "1.13"

-------------------------------------------------
-- PARAMETER TABLE
--   PARAM_TABLE_KEY must be unique across every Lua script on this vehicle.
--   Changing it orphans the previously saved values.
-------------------------------------------------
local PARAM_TABLE_KEY    = 73
local PARAM_TABLE_PREFIX = "TANK_"
-- Raised from 32 for the v1.11 flight speed params. Safe: the table key's
-- CRC is taken over the prefix alone, so resizing never orphans saved values.
-- Index 34 is free but unused: it briefly held a fast-mode speed, which
-- LOIT_SPEED already is. Indices 27 and 28 are RETIRED. They held TANK_FLY_SLOW / TANK_FLY_FAST,
-- which were stick fractions; v1.13 sets flight speed in cm/s instead. The
-- old values are still in EEPROM at those indices, so do not reuse them: a
-- new parameter placed at 27 would silently inherit 0.293.
local PARAM_TABLE_SIZE   = 40

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, PARAM_TABLE_SIZE),
       SCRIPT_NAME .. ": could not add param table")

local function bind_param(idx, name, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
           SCRIPT_NAME .. ": could not add param " .. name)
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

--[[
  // @Param: TANK_PIT_CH
  // @DisplayName: Tank throttle input channel
  // @Description: RC input channel read as forward/reverse throttle while in tank mode. This is the left stick up/down axis on a split-arcade transmitter, normally the Pitch channel.
  // @Range: 1 16
  // @Increment: 1
  // @User: Standard
--]]
local TANK_PIT_CH    = bind_param( 1, "PIT_CH",    2)

--[[
  // @Param: TANK_YAW_CH
  // @DisplayName: Tank steering input channel
  // @Description: RC input channel read as left/right steering while in tank mode. This is the right stick left/right axis on a split-arcade transmitter, normally the Yaw channel.
  // @Range: 1 16
  // @Increment: 1
  // @User: Standard
--]]
local TANK_YAW_CH    = bind_param( 2, "YAW_CH",    4)

--[[
  // @Param: TANK_MODE_CH
  // @DisplayName: Tank mode select channel
  // @Description: RC channel that selects tank mode. Tank mode engages when this channel reads above TANK_MODE_TRH, and only while the vehicle is disarmed.
  // @Range: 1 16
  // @Increment: 1
  // @User: Standard
--]]
local TANK_MODE_CH   = bind_param( 3, "MODE_CH",  12)

--[[
  // @Param: TANK_SPD_CH
  // @DisplayName: Tank speed select channel
  // @Description: RC channel that selects the speed scale. Below TANK_SPD_TRH the slow scale TANK_SLOW_SCL is applied, otherwise TANK_FAST_SCL is applied.
  // @Range: 1 16
  // @Increment: 1
  // @User: Standard
--]]
local TANK_SPD_CH    = bind_param( 4, "SPD_CH",    5)

--[[
  // @Param: TANK_MODE_TRH
  // @DisplayName: Tank mode PWM threshold
  // @Description: Tank mode engages when TANK_MODE_CH reads above this pulse width, and reverts to drone mode at or below it.
  // @Units: PWM
  // @Range: 900 2100
  // @Increment: 1
  // @User: Standard
--]]
local TANK_MODE_TRH  = bind_param( 5, "MODE_TRH", 1500)

--[[
  // @Param: TANK_SPD_TRH
  // @DisplayName: Slow speed PWM threshold
  // @Description: The slow scale TANK_SLOW_SCL is applied when TANK_SPD_CH reads below this pulse width. Above it, TANK_FAST_SCL is applied.
  // @Units: PWM
  // @Range: 900 2100
  // @Increment: 1
  // @User: Standard
--]]
local TANK_SPD_TRH   = bind_param( 6, "SPD_TRH",  1200)

--[[
  // @Param: TANK_SLOW_SCL
  // @DisplayName: Slow mode output scale
  // @Description: Fraction of full stick travel passed to the tracks in slow mode. Scales throttle and steering together, so it limits top speed without changing turn balance.
  // @Range: 0 1
  // @Increment: 0.01
  // @User: Standard
--]]
local TANK_SLOW_SCL  = bind_param( 7, "SLOW_SCL", 0.35)

--[[
  // @Param: TANK_FAST_SCL
  // @DisplayName: Fast mode output scale
  // @Description: Fraction of full stick travel passed to the tracks in fast mode. Values below 1 keep both tracks off their end stops, which leaves the per-side gains room to correct a speed mismatch at full throttle.
  // @Range: 0 1
  // @Increment: 0.01
  // @User: Standard
--]]
local TANK_FAST_SCL  = bind_param( 8, "FAST_SCL", 1.0)

--[[
  // @Param: TANK_L_GAIN
  // @DisplayName: Left track speed match gain
  // @Description: Multiplier on the left track output magnitude, used to match track speeds so the vehicle drives straight. Trim the faster track down rather than raising the slower one above 1, which would clip.
  // @Range: 0 1
  // @Increment: 0.005
  // @User: Standard
--]]
local TANK_L_GAIN    = bind_param( 9, "L_GAIN",   0.885)

--[[
  // @Param: TANK_R_GAIN
  // @DisplayName: Right track speed match gain
  // @Description: Multiplier on the right track output magnitude, used to match track speeds so the vehicle drives straight. Trim the faster track down rather than raising the slower one above 1, which would clip.
  // @Range: 0 1
  // @Increment: 0.005
  // @User: Standard
--]]
local TANK_R_GAIN    = bind_param(10, "R_GAIN",   1.0)

--[[
  // @Param: TANK_L_OFS
  // @DisplayName: Left track break-away offset
  // @Description: Pulse width added to the left track output magnitude whenever the stick is off centre, to jump the motor's dead band so the track starts moving immediately. Applied in the track's own direction of travel. Negative values reduce the output instead, which lets the gain and offset together match track speeds at both low and high throttle. Zero disables.
  // @Units: PWM
  // @Range: -200 200
  // @Increment: 1
  // @User: Standard
--]]
local TANK_L_OFS     = bind_param(11, "L_OFS",    0)

--[[
  // @Param: TANK_R_OFS
  // @DisplayName: Right track break-away offset
  // @Description: Pulse width added to the right track output magnitude whenever the stick is off centre, to jump the motor's dead band so the track starts moving immediately. Applied in the track's own direction of travel. Negative values reduce the output instead, which lets the gain and offset together match track speeds at both low and high throttle. Zero disables.
  // @Units: PWM
  // @Range: -200 200
  // @Increment: 1
  // @User: Standard
--]]
local TANK_R_OFS     = bind_param(12, "R_OFS",    0)

--[[
  // @Param: TANK_OUT_L_CH
  // @DisplayName: Left track output channel
  // @Description: RC channel this script overrides to drive the left track. Takes effect immediately, no reboot needed.
  // @Range: 1 16
  // @Increment: 1
  // @User: Standard
--]]
local TANK_OUT_L_CH  = bind_param(13, "OUT_L_CH", 13)

--[[
  // @Param: TANK_OUT_R_CH
  // @DisplayName: Right track output channel
  // @Description: RC channel this script overrides to drive the right track. Takes effect immediately, no reboot needed.
  // @Range: 1 16
  // @Increment: 1
  // @User: Standard
--]]
local TANK_OUT_R_CH  = bind_param(14, "OUT_R_CH", 14)

--[[
  // @Param: TANK_OUT_P_CH
  // @DisplayName: Drone pitch passthrough channel
  // @Description: RC channel the throttle input is copied to while in drone mode, so the same stick flies the aircraft when tank mode is off.
  // @Range: 1 16
  // @Increment: 1
  // @User: Standard
--]]
local TANK_OUT_P_CH  = bind_param(15, "OUT_P_CH", 15)

--[[
  // @Param: TANK_OUT_Y_CH
  // @DisplayName: Drone yaw passthrough channel
  // @Description: RC channel the steering input is copied to while in drone mode, so the same stick flies the aircraft when tank mode is off.
  // @Range: 1 16
  // @Increment: 1
  // @User: Standard
--]]
local TANK_OUT_Y_CH  = bind_param(16, "OUT_Y_CH", 16)

--[[
  // @Param: TANK_IN_CTR
  // @DisplayName: Input centre pulse width
  // @Description: Pulse width treated as stick centre when converting the throttle and steering inputs to a normalised demand. Set this to the RCn_TRIM of your input channels, otherwise a centred stick reads as a small steering demand and the tracks creep or pull.
  // @Units: PWM
  // @Range: 1400 1600
  // @Increment: 1
  // @User: Standard
--]]
local TANK_IN_CTR    = bind_param(17, "IN_CTR",   1500)

--[[
  // @Param: TANK_IN_RNG
  // @DisplayName: Input half range pulse width
  // @Description: Pulse width from centre to full stick deflection. Set this to RCn_MAX minus RCn_TRIM so full stick maps to exactly full output. Lowering it makes the sticks more sensitive but saturates the mixer before full deflection.
  // @Units: PWM
  // @Range: 100 600
  // @Increment: 1
  // @User: Standard
--]]
local TANK_IN_RNG    = bind_param(18, "IN_RNG",   450)

--[[
  // @Param: TANK_PIT_REV
  // @DisplayName: Reverse throttle input
  // @Description: Mirrors the throttle input about TANK_IN_CTR. Enable this if pushing the stick forward drives the vehicle backwards.
  // @Values: 0:Normal,1:Reversed
  // @User: Standard
--]]
local TANK_PIT_REV   = bind_param(19, "PIT_REV",  1)

--[[
  // @Param: TANK_RATE_MS
  // @DisplayName: Script update period
  // @Description: How often the script re-reads the sticks and writes the track outputs. Lower values give crisper control at the cost of more scripting CPU.
  // @Units: ms
  // @Range: 10 1000
  // @Increment: 10
  // @User: Advanced
--]]
local TANK_RATE_MS   = bind_param(20, "RATE_MS",  100)

--[[
  // @Param: TANK_EKF_MAN
  // @DisplayName: Manage EKF3 source set
  // @Description: When enabled, the script selects which EK3_SRC set the EKF uses on every mode change: EK3_SRC3 while driving in tank mode, and the set chosen by TANK_NAV_SRC on returning to drone mode. Only the active selection changes, never the EK3_SRC parameter values themselves, and the selection is not saved so a power cycle always returns to EK3_SRC1. Disable to leave EKF source selection entirely alone.
  // @Values: 0:Disabled,1:Enabled
  // @User: Advanced
--]]
local TANK_EKF_MAN   = bind_param(21, "EKF_MAN",  1)

--[[
  // @Param: TANK_NAV_SRC
  // @DisplayName: Drone mode navigation source
  // @Description: Which EKF3 source set is selected for drone mode. Optical Flow selects EK3_SRC2 and SLAM selects EK3_SRC1, so configure the actual sensor choices in those parameters. Applied when leaving tank mode and once at startup. Ignored while tank mode is engaged, where EK3_SRC3 is selected instead. Requires TANK_EKF_MAN to be enabled.
  // @Values: 0:Optical Flow (EK3_SRC2),1:SLAM (EK3_SRC1)
  // @User: Standard
--]]
local TANK_NAV_SRC   = bind_param(22, "NAV_SRC",  1)

--[[
  // @Param: TANK_L_REV
  // @DisplayName: Reverse left track
  // @Description: Flips the direction of the left track output. SERVOn_REVERSED has no effect on outputs set to RCIN passthrough, so track direction must be corrected here or in the motor wiring.
  // @Values: 0:Normal,1:Reversed
  // @User: Standard
--]]
local TANK_L_REV     = bind_param(23, "L_REV",    0)

--[[
  // @Param: TANK_R_REV
  // @DisplayName: Reverse right track
  // @Description: Flips the direction of the right track output. If throttle makes the vehicle rotate and steering makes it drive straight, the two tracks share the same polarity instead of being mirrored, and exactly one of TANK_L_REV or TANK_R_REV needs setting.
  // @Values: 0:Normal,1:Reversed
  // @User: Standard
--]]
local TANK_R_REV     = bind_param(24, "R_REV",    0)

--[[
  // @Param: TANK_YAW_REV
  // @DisplayName: Reverse steering
  // @Description: Mirrors the steering demand, so a right stick rotates the other way. Use when forward and reverse are already correct but rotation is backwards. TANK_L_REV and TANK_R_REV cannot fix this on their own, because negating a track flips forward/reverse and rotation together.
  // @Values: 0:Normal,1:Reversed
  // @User: Standard
--]]
local TANK_YAW_REV   = bind_param(25, "YAW_REV",  0)

-------------------------------------------------
-- FLIGHT SPEED SELECT (v1.11)
--   Reuses TANK_SPD_CH / TANK_SPD_TRH, so one switch sets both the driving
--   speed and the flying speed.
-------------------------------------------------

--[[
  // @Param: TANK_FLY_EN
  // @DisplayName: Enable flight speed select
  // @Description: When enabled, the tank speed switch also selects the flying speed in drone mode by scaling the pilot roll and pitch input. When disabled the script passes every stick straight through, exactly as v1.10 did. Roll is still routed through the script either way, so the aircraft stays flyable if this is turned off after RCMAP_ROLL has been pointed at TANK_OUT_RL_CH.
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local TANK_FLY_EN    = bind_param(26, "FLY_EN",    0)

--[[
  // @Param: TANK_SPD_HYST
  // @DisplayName: Speed switch hysteresis
  // @Description: Pulse width either side of TANK_SPD_TRH that the speed switch must cross before the selection changes, in both tank and flight modes. This is not a delay: it widens the travel needed to change state, so a switch resting near the threshold cannot chatter between slow and fast. A three position switch sits 150 or more from a threshold of 1200, so the default costs it nothing. Lower it only if you select speed with a knob or a slider and want it to change closer to the threshold. Zero disables hysteresis and restores a bare compare against TANK_SPD_TRH.
  // @Units: PWM
  // @Range: 0 200
  // @Increment: 5
  // @User: Standard
--]]
local TANK_SPD_HYST  = bind_param(32, "SPD_HYST",  25)

--[[
  // @Param: TANK_FLY_SLOW_V
  // @DisplayName: Slow flight speed
  // @Description: Ground speed at full roll or pitch stick in slow mode, in cm/s. The script converts this to a stick scale by inverting Loiter's steady state, v = LOIT_SPEED * tan(LOIT_ANG_MAX * n) / tan(LOIT_ANG_MAX), and then accounts for the output channel's dead zone, which the flight code subtracts before normalising. LOIT_SPEED, LOIT_ANG_MAX and the channel's TRIM, MAX and DZ are all read live, so retuning any of them keeps this speed correct instead of invalidating it. Values at or above LOIT_SPEED give full stick. Fast mode is always full stick, so its speed is LOIT_SPEED itself and has no parameter here. This is an open loop mapping, not a closed loop speed limit: it sets where the aircraft settles with the stick held over, and it assumes LOIT_SPEED is the binding limit, which it is not if the EKF is imposing a lower ground speed limit of its own.
  // @Units: cm/s
  // @Range: 20 2000
  // @Increment: 10
  // @User: Standard
--]]
local TANK_FLY_SLOW_V = bind_param(33, "FLY_SLOW_V", 100)


--[[
  // @Param: TANK_ROL_CH
  // @DisplayName: Roll input channel
  // @Description: RC input channel read as roll while in drone mode. This is the transmitter's own roll stick, normally channel 1.
  // @Range: 1 16
  // @Increment: 1
  // @User: Standard
--]]
local TANK_ROL_CH    = bind_param(29, "ROL_CH",    1)

--[[
  // @Param: TANK_OUT_RL_CH
  // @DisplayName: Roll output channel
  // @Description: RC channel the roll input is copied to while in drone mode, so the script can scale it. Zero leaves roll alone entirely, which is the default so that installing this script changes nothing until you ask it to. To use the flight speed select, set this to a spare channel, point RCMAP_ROLL at the same channel, and reboot. Pick one the receiver actually sends and leaves centred, so roll falls back to neutral rather than to a hard deflection if the script ever stops. Set the channel's RCn_TRIM to 1500, RCn_MIN to 1050, RCn_MAX to 1950 and RCn_DZ to 20, matching the pitch and yaw output channels. Roll stays routed even when TANK_FLY_EN is off, so turning the feature off never leaves the aircraft without roll.
  // @Range: 0 16
  // @Increment: 1
  // @User: Standard
--]]
local TANK_OUT_RL_CH = bind_param(30, "OUT_RL_CH", 0)

-------------------------------------------------
-- EKF3 SOURCE SETS
--   This script only SELECTS between the source sets. It never writes the
--   EK3_SRC* values, so your configuration in Mission Planner stays
--   authoritative and cannot be clobbered. The selection is runtime only and
--   is not persisted: a reboot always comes back on set 1, which is why
--   powering down in tank mode can no longer leave an unbootable EKF config.
--
--   Expected contents (configure these in Mission Planner, not here):
--     EK3_SRC1_*  SLAM         POSXY 6  VELXY 6  POSZ 6  VELZ 6  YAW 6
--     EK3_SRC2_*  OpticalFlow  POSXY 0  VELXY 5  POSZ 2  VELZ 0  YAW 1
--     EK3_SRC3_*  Tank         POSXY 0  VELXY 0  POSZ 0  VELZ 0  YAW 0
-------------------------------------------------
local SRC_SET_SLAM = 1      -- EK3_SRC1_*
local SRC_SET_FLOW = 2      -- EK3_SRC2_*
local SRC_SET_TANK = 3      -- EK3_SRC3_*

-- ahrs:set_posvelyaw_source_set() is zero-based, so set 1 is index 0.
-- If it turns out to be one-based on your firmware, change this to 0 and
-- every mapping below still holds.
local SRC_SET_OFFSET = -1

-- Kept terse on purpose: MAVLink STATUSTEXT truncates at 50 characters.
-- Since v1.12 these ride behind "TANK"/"FLIGHT" rather than the 17-character
-- SCRIPT_NAME, so there is headroom now -- but lengthening them still eats the
-- message silently, with no error, so keep them short.
local SRC_SET_NAMES = {
    [SRC_SET_SLAM] = "1 SLAM",
    [SRC_SET_FLOW] = "2 Flow",
    [SRC_SET_TANK] = "3 None",
}

-- Set once if the binding is missing, so we warn a single time and then let
-- the driving logic carry on rather than killing the whole script.
local src_set_broken = false

local function apply_source_set(set_num)
    if src_set_broken then return false end
    local idx = set_num + SRC_SET_OFFSET
    local ok = pcall(function() ahrs:set_posvelyaw_source_set(idx) end)
    if not ok then
        src_set_broken = true
        gcs:send_text(3, SCRIPT_NAME .. ": src set API missing")
    end
    return ok
end

-- The drone-mode source set currently selected by TANK_NAV_SRC.
local function selected_drone_set()
    if math.floor(TANK_NAV_SRC:get() or 1) == 0 then
        return SRC_SET_FLOW
    end
    return SRC_SET_SLAM
end

-------------------------------------------------
-- PREARM BLOCK
--   Registers an auxiliary arming authorisation so the vehicle cannot arm at
--   all while tank mode is engaged. This is a genuine prearm failure reported
--   by the GCS, replacing the old behaviour of letting the arm succeed and
--   disarming a fraction of a second later with the motors already spinning.
--   The forced disarm below is kept as a backstop.
-------------------------------------------------
local auth_id      = nil    -- authorisation slot, claimed once at init
local auth_blocked = nil    -- nil forces the first publish

local function claim_arming_auth()
    local ok, id = pcall(function() return arming:get_aux_auth_id() end)
    if ok then auth_id = id end
    if auth_id == nil then
        gcs:send_text(4, SCRIPT_NAME .. ": no prearm auth slot")
    end
end

local function set_arming_block(blocked)
    if auth_id == nil then return end
    if blocked == auth_blocked then return end   -- publish only on change
    local ok
    if blocked then
        ok = pcall(function()
            arming:set_aux_auth_failed(auth_id, "tank mode engaged")
        end)
    else
        ok = pcall(function() arming:set_aux_auth_passed(auth_id) end)
    end
    if not ok then
        auth_id = nil                            -- stop trying, warn once
        gcs:send_text(4, SCRIPT_NAME .. ": prearm auth API failed")
        return
    end
    auth_blocked = blocked
end

-------------------------------------------------
-- OUTPUT CHANNEL BINDINGS (re-bound only when the param changes)
-------------------------------------------------
local function make_channel_binding(p)
    local ch, obj = nil, nil
    return function()
        local want = math.floor(p:get() or 0)
        if want < 1 or want > 16 then return nil end
        if want ~= ch then
            ch, obj = want, rc:get_channel(want)
        end
        return obj
    end
end

local get_track_left  = make_channel_binding(TANK_OUT_L_CH)
local get_track_right = make_channel_binding(TANK_OUT_R_CH)
local get_drone_pitch = make_channel_binding(TANK_OUT_P_CH)
local get_drone_yaw   = make_channel_binding(TANK_OUT_Y_CH)
local get_drone_roll  = make_channel_binding(TANK_OUT_RL_CH)

-------------------------------------------------
-- RC PARAMETER BINDINGS
--   Binds RC<n>_<SUFFIX> for whichever channel a TANK_ param currently names,
--   rebinding only when that channel number changes.
--
--   Reading the trims from AP rather than carrying our own copy makes the
--   radio calibration the single source of truth. Recalibrate the radio and
--   the scaling follows, with nothing to keep in step by hand, and each axis
--   is scaled about its own rest position rather than a shared guess.
-------------------------------------------------
local function make_rc_param_binding(chan_param, suffix)
    local ch, p = nil, nil
    return function()
        local want = math.floor(chan_param:get() or 0)
        if want < 1 or want > 16 then
            ch, p = nil, nil
            return nil
        end
        if want ~= ch then
            ch = want
            local np = Parameter()
            p = np:init("RC" .. want .. "_" .. suffix) and np or nil
        end
        if p == nil then return nil end
        return p:get()
    end
end

-- Grouped per axis: converting a speed to a scale needs six of these at once,
-- and passing them positionally was already at the edge of readable.
local roll_rc = {
    in_trim  = make_rc_param_binding(TANK_ROL_CH,    "TRIM"),
    in_min   = make_rc_param_binding(TANK_ROL_CH,    "MIN"),
    in_max   = make_rc_param_binding(TANK_ROL_CH,    "MAX"),
    out_trim = make_rc_param_binding(TANK_OUT_RL_CH, "TRIM"),
    out_min  = make_rc_param_binding(TANK_OUT_RL_CH, "MIN"),
    out_max  = make_rc_param_binding(TANK_OUT_RL_CH, "MAX"),
    out_dz   = make_rc_param_binding(TANK_OUT_RL_CH, "DZ"),
}
local pitch_rc = {
    in_trim  = make_rc_param_binding(TANK_PIT_CH,   "TRIM"),
    in_min   = make_rc_param_binding(TANK_PIT_CH,   "MIN"),
    in_max   = make_rc_param_binding(TANK_PIT_CH,   "MAX"),
    out_trim = make_rc_param_binding(TANK_OUT_P_CH, "TRIM"),
    out_min  = make_rc_param_binding(TANK_OUT_P_CH, "MIN"),
    out_max  = make_rc_param_binding(TANK_OUT_P_CH, "MAX"),
    out_dz   = make_rc_param_binding(TANK_OUT_P_CH, "DZ"),
}

-- Flight-code parameters the speed conversion depends on. Bound lazily and
-- cached, the same way the RC bindings are, so a missing one degrades to nil
-- rather than erroring at load on a vehicle that does not have it.
local function make_ap_param_binding(name)
    local p, tried = nil, false
    return function()
        if not tried then
            tried = true
            local np = Parameter()
            p = np:init(name) and np or nil
        end
        if p == nil then return nil end
        return p:get()
    end
end

local get_loit_speed   = make_ap_param_binding("LOIT_SPEED")     -- cm/s
local get_loit_ang_max = make_ap_param_binding("LOIT_ANG_MAX")   -- deg, 0 = derive
local get_angle_max    = make_ap_param_binding("ANGLE_MAX")      -- centidegrees
local get_psc_ang_max  = make_ap_param_binding("PSC_ANGLE_MAX")  -- deg, 0 = ANGLE_MAX

-------------------------------------------------
-- HELPERS
-------------------------------------------------
local function clamp(v, lo, hi)
    if v < lo then return lo elseif v > hi then return hi end
    return v
end

local function pct_to_pwm(v)            -- -1..+1 -> 1000..2000
    return math.floor(clamp(v, -1, 1) * 500 + 1500 + 0.5)
end

-- Trim in signed PWM-delta space so per-side offset always pushes the track
-- in ITS OWN direction of travel, whichever way that track is reversed.
local function trim(pwm, gain, ofs)
    local d = pwm - 1500
    if d == 0 then return 1500 end          -- neutral stays neutral, no creep
    local s = (d > 0) and 1 or -1
    d = s * (ofs + math.abs(d) * gain)
    return math.floor(1500 + clamp(d, -500, 500) + 0.5)
end

-------------------------------------------------
-- FLIGHT SPEED SCALING
-------------------------------------------------
-- Two different centres are in play and they are not the same number.
--   The INPUT centre is where the pilot's stick physically rests, which is
--   by definition the input channel's RCn_TRIM from the radio calibration.
--   The OUTPUT centre is what the flight code treats as zero demand, i.e.
--   the RCn_TRIM of the mapped output channel.
-- Scaling about the input centre and emitting about the output centre makes a
-- centred stick land on exactly the output trim at every scale, instead of
-- relying on the output channel's dead zone to swallow the difference.
--
-- Scale one axis onto its output channel. Returns false if any RC parameter
-- it needs could not be read, so the caller can fall back to passing the
-- stick through untouched: the axis keeps working and the pilot is told the
-- speed limit is not being applied.
-- The lean angle Loiter allows the pilot, mirroring AC_Loiter::get_angle_max_cd
-- and AC_PosControl::get_lean_angle_max_cd. Degrees, or nil if unreadable.
local function loiter_angle_max_deg()
    local amax = get_angle_max()                       -- centidegrees
    if amax == nil then return nil end
    local psc  = get_psc_ang_max()                     -- degrees, 0 = ANGLE_MAX
    local psc_cd = (psc ~= nil and psc > 0) and (psc * 100.0) or amax
    local loit = get_loit_ang_max()                    -- degrees, 0 = derive
    if loit ~= nil and loit > 0 then
        return math.min(loit * 100.0, psc_cd) * 0.01
    end
    -- LOIT_ANG_MAX 0: Loiter takes two thirds of the smaller ceiling.
    return math.min(amax, psc_cd) * (2.0/3.0) * 0.01
end

-- Fraction of full stick that settles at v_cms in Loiter.
--
-- Loiter's drag term is  accel(ang_max) * v / LOIT_SPEED  (AC_Loiter.cpp), and
-- the pilot's stick commands  accel(ang_max * n)  because a single-axis stick
-- maps linearly to lean angle (control.cpp: atan(tan(ang_max * n)) is ang_max
-- * n). Setting those equal, with accel(x) = g * tan(x):
--     v = LOIT_SPEED * tan(ang_max * n) / tan(ang_max)
-- which inverts to the line below. Returns 1.0 for "as fast as Loiter allows",
-- or nil if the flight parameters it needs cannot be read.
local function stick_fraction_for(v_cms)
    local vmax = get_loit_speed()
    local ang  = loiter_angle_max_deg()
    if vmax == nil or ang == nil or vmax <= 0 or ang <= 0 then
        return nil
    end
    if v_cms == nil or v_cms <= 0 or v_cms >= vmax then
        return 1.0
    end
    local a = math.rad(ang)
    return math.atan((v_cms / vmax) * math.tan(a)) / a
end

-- Scale one axis onto its output channel so that full stick settles at v_cms.
-- Returns false if any parameter it needs could not be read, so the caller can
-- fall back to passing the stick through untouched: the axis keeps working and
-- the pilot is told the speed limit is not being applied.
local function drive_axis(in_pwm, out_chan, v_cms, rc)
    if in_pwm == nil or out_chan == nil then
        return true                     -- nothing to drive, nothing wrong
    end
    local n       = stick_fraction_for(v_cms)
    local in_ctr  = rc.in_trim()
    local out_ctr = rc.out_trim()
    local out_min = rc.out_min()
    local out_max = rc.out_max()
    local out_dz  = rc.out_dz()
    if n == nil or not (in_ctr and out_ctr and out_min and out_max and out_dz) then
        return false
    end

    -- Each direction gets its own travel. Radio calibrations are rarely
    -- symmetric, and the flight code normalises each side separately.
    local dev = in_pwm - in_ctr
    local in_end, out_end
    if dev < 0 then
        in_end, out_end = rc.in_min(), out_min
    else
        in_end, out_end = rc.in_max(), out_max
    end
    if not (in_end and out_end) then
        return false
    end

    local in_travel  = math.abs(in_end - in_ctr)
    -- norm_input_dz() measures from TRIM + DZ, so the dead zone is taken out of
    -- the usable travel before the tangent mapping. Adding it back here is what
    -- makes the requested speed the one actually flown.
    local out_travel = math.abs(out_end - out_ctr) - out_dz
    if in_travel < 1 or out_travel < 1 then
        return false
    end

    local out = out_ctr + dev * ((n * out_travel + out_dz) / in_travel)
    out_chan:set_override(math.floor(clamp(out, out_min, out_max) + 0.5))
    return true
end

-- Rate limit for the "cannot read the RC params" warning. nil means never
-- warned, so the first one is immediate rather than being swallowed by the
-- rate limit during the first few seconds after boot.
local last_rc_warn_ms = nil
local function warn_rc_params()
    local now = millis():toint()
    if last_rc_warn_ms == nil or now - last_rc_warn_ms > 5000 or now < last_rc_warn_ms then
        gcs:send_text(4, SCRIPT_NAME .. ": RC trims unreadable, speed limit off")
        last_rc_warn_ms = now
    end
end

-- nil = undecided, so the first pass adopts and announces whichever position
-- the switch is already sitting on rather than assuming one.
local speed_slow      = nil
-- Which mode the last announcement was about. A mode change re-announces:
-- the switch has not moved, but what it now selects has, so a pilot who saw
-- "TANK: speed SLOW" must not be left assuming it still describes the vehicle.
local announced_tank  = nil

-- ONE switch, TWO meanings: below TANK_SPD_TRH it selects TANK_SLOW_SCL for
-- the tracks and TANK_FLY_SLOW_V for the flight sticks. Decided once per loop
-- so the tracks and the sticks can never disagree about which side of the
-- hysteresis band the switch is on.
--   tank     - true if tank mode is engaged; picks the wording.
--   announce - false suppresses the message without changing the decision,
--              for when the selected scale is not actually being applied.
-- Returns true for slow.
local function update_speed_select(speed_pwm, tank, announce)
    local trh  = TANK_SPD_TRH:get() or 1200
    local hyst = math.max(TANK_SPD_HYST:get() or 25, 0)
    local want = speed_slow
    if speed_pwm == nil then
        -- No reading from the switch. Slow is the conservative answer.
        want = true
    elseif speed_pwm < trh - hyst then
        want = true
    elseif speed_pwm > trh + hyst then
        want = false
    elseif want == nil then
        -- First pass, and the switch is inside the hysteresis band.
        want = speed_pwm < trh
    end

    local changed = (want ~= speed_slow) or (tank ~= announced_tank)
    speed_slow = want
    if changed and announce then
        announced_tank = tank
        gcs:send_text(6, (tank and "TANK" or "FLIGHT") .. ": speed "
                         .. (speed_slow and "SLOW" or "FAST"))
    elseif changed and not announce then
        -- Left unannounced, so the next announcement is not suppressed as a
        -- repeat of something the pilot was never told.
        announced_tank = nil
    end
    return speed_slow
end

-- The speed to fly at full stick, in cm/s, or nil when the feature is disabled
-- and the sticks should pass straight through. Zero is a real value here --
-- it means full stick -- so callers must test for nil, not for truth.
local function fly_speed_target(slow)
    if (TANK_FLY_EN:get() or 0) <= 0 then
        return nil
    end
    if slow then
        return TANK_FLY_SLOW_V:get() or 100
    end
    -- Fast is full stick, which is LOIT_SPEED by definition. There is no
    -- parameter for it: holding fast mode below LOIT_SPEED is what LOIT_SPEED
    -- already does, and a second ceiling could only disagree with the first.
    return 0
end

-------------------------------------------------
-- STATE TRACKING
--   nil = not yet determined, so the first pass selects the source set
--   matching the current switch position. That is what makes TANK_NAV_SRC
--   take effect at startup rather than only after a tank mode toggle.
-------------------------------------------------
local in_tank_mode = nil

-- Rate limit for the "you are armed" refusal, so it cannot spam at loop rate.
local last_refuse_ms = 0

-------------------------------------------------
-- MAIN LOOP
-------------------------------------------------
local function update()
    local rate_ms    = math.floor(clamp(TANK_RATE_MS:get() or 100, 10, 1000))
    local in_ctr     = TANK_IN_CTR:get() or 1500
    local in_rng     = TANK_IN_RNG:get() or 450
    if in_rng < 1 then in_rng = 1 end
    local manage_ekf = (TANK_EKF_MAN:get() or 1) > 0

    local mode_pwm  = rc:get_pwm(math.floor(TANK_MODE_CH:get() or 12))
    local speed_pwm = rc:get_pwm(math.floor(TANK_SPD_CH:get()  or 5))
    local t_pwm     = rc:get_pwm(math.floor(TANK_PIT_CH:get()  or 2))
    local s_pwm     = rc:get_pwm(math.floor(TANK_YAW_CH:get()  or 4))
    local r_pwm     = rc:get_pwm(math.floor(TANK_ROL_CH:get()  or 1))

    local track_left  = get_track_left()
    local track_right = get_track_right()
    local drone_pitch = get_drone_pitch()
    local drone_yaw   = get_drone_yaw()
    local drone_roll  = get_drone_roll()

    -- STATE TRANSITIONS
    local want_tank = mode_pwm and mode_pwm > (TANK_MODE_TRH:get() or 1500)
    if want_tank then
        if in_tank_mode ~= true then
            if arming:is_armed() then
                -- Refuse, but say so. A silent no-op here is indistinguishable
                -- from a dead switch or a script that has stopped running.
                local now = millis():toint()
                if now - last_refuse_ms > 3000 or now < last_refuse_ms then
                    gcs:send_text(4, SCRIPT_NAME .. ": armed, tank mode blocked")
                    last_refuse_ms = now
                end
            else
                if manage_ekf then apply_source_set(SRC_SET_TANK) end
                gcs:send_text(6, "TANK ON, src "
                                 .. SRC_SET_NAMES[SRC_SET_TANK])
                in_tank_mode = true
            end
        end
    else
        if in_tank_mode ~= false then
            local set_num = selected_drone_set()
            if manage_ekf then apply_source_set(set_num) end
            gcs:send_text(6, "FLIGHT ON, src "
                             .. SRC_SET_NAMES[set_num])
            in_tank_mode = false
        end
    end

    -- Arming is blocked for as long as tank mode is engaged.
    set_arming_block(in_tank_mode == true)

    -- One switch, decided once, so the tracks and the sticks cannot end up on
    -- opposite sides of the hysteresis band. Announced whenever the selection
    -- is actually applied: always in tank mode, and in flight only when
    -- TANK_FLY_EN is on -- with it off the sticks pass through unscaled and
    -- announcing a flight speed would describe something that is not happening.
    local tank = in_tank_mode == true
    local slow = update_speed_select(speed_pwm, tank,
                                     tank or (TANK_FLY_EN:get() or 0) > 0)
    local scale
    if slow then
        scale = TANK_SLOW_SCL:get() or 0.35
    else
        scale = TANK_FAST_SCL:get() or 1.0
    end

    -- OUTPUTS
    if in_tank_mode then
        if arming:is_armed() then arming:disarm() end

        if t_pwm and s_pwm and track_left and track_right then
            if (TANK_PIT_REV:get() or 1) > 0 then
                t_pwm = 2 * in_ctr - t_pwm
            end
            local throttle = ((t_pwm - in_ctr) / in_rng) * scale
            local steer    = ((s_pwm - in_ctr) / in_rng) * scale
            -- Steer sign is independent of track polarity: this flips rotation
            -- while leaving forward/reverse alone.
            if (TANK_YAW_REV:get() or 0) > 0 then steer = -steer end

            -- Both tracks take throttle with the SAME sign and steer with
            -- OPPOSITE signs. Whether a track then needs inverting depends on
            -- how its motor is wired, which is what L_REV / R_REV are for.
            local l_sign = ((TANK_L_REV:get() or 0) > 0) and -1 or 1
            local r_sign = ((TANK_R_REV:get() or 0) > 0) and -1 or 1
            local left  = l_sign * (throttle - steer)
            local right = r_sign * (throttle + steer)

            track_left :set_override(trim(pct_to_pwm(left),
                                          TANK_L_GAIN:get() or 1.0,
                                          TANK_L_OFS:get()  or 0))
            track_right:set_override(trim(pct_to_pwm(right),
                                          TANK_R_GAIN:get() or 1.0,
                                          TANK_R_OFS:get()  or 0))
        end
    else
        -- Roll and pitch are routed through the script so the flight speed
        -- select can scale them. Yaw is passed straight through: it sets a
        -- rotation rate, not a speed over the ground.
        local v_cms = fly_speed_target(slow)
        if v_cms ~= nil then
            local ok_p = drive_axis(t_pwm, drone_pitch, v_cms, pitch_rc)
            local ok_r = drive_axis(r_pwm, drone_roll,  v_cms, roll_rc)
            if not (ok_p and ok_r) then
                warn_rc_params()
                if t_pwm and drone_pitch then drone_pitch:set_override(t_pwm) end
                if r_pwm and drone_roll  then drone_roll :set_override(r_pwm) end
            end
        else
            -- Disabled: v1.10 behaviour, every axis passed through untouched.
            if t_pwm and drone_pitch then drone_pitch:set_override(t_pwm) end
            if r_pwm and drone_roll  then drone_roll :set_override(r_pwm) end
        end
        if s_pwm and drone_yaw   then drone_yaw  :set_override(s_pwm) end
        if track_left  then track_left :set_override(1500) end
        if track_right then track_right:set_override(1500) end
    end

    return update, rate_ms
end

-------------------------------------------------
-- INIT
-------------------------------------------------
local function init()
    gcs:send_text(6, SCRIPT_NAME .. " " .. VERSION .. " init, src "
                     .. SRC_SET_NAMES[selected_drone_set()])
    -- Claim the slot and immediately publish "passed", otherwise the vehicle
    -- sits in "auth not received" until the first update pass.
    claim_arming_auth()
    set_arming_block(false)
    local l, r = get_track_left(), get_track_right()
    if l then l:set_override(1500) end
    if r then r:set_override(1500) end
    -- Claim the roll output at neutral straight away, so RCMAP_ROLL never
    -- reads a stale or unset value in the window before the first update.
    local rl = get_drone_roll()
    if rl then rl:set_override(math.floor(roll_rc.out_trim() or 1500)) end
    return update, 1000
end

return init()

/****************************************************************
 * tof5x_mavlink_v4.1 — QT Py SAMD21 + TCA9548A + 5 x VL53L5CX (4x4 grid)
 *
 * v4.4 changes over v4.3:
 *   - SENSOR MASK back to the v4.2 set: TOP off (not used for collision
 *     avoidance), everything else on. v4.3 shipped with LEFT disabled and
 *     RIGHT enabled, which is backwards from the 2026-09-07 flight evidence:
 *     both boots came up 4/4 healthy, then RIGHT went LOST in both flights
 *     while LEFT went LOST in the first and recovered by the second.
 *     RIGHT remains enabled by operator decision, against the warning below.
 *   - BLOCKING TIME, which is what turns a sick sensor into a board reboot:
 *       * one reinit per loop iteration is now actually enforced. Two paths
 *         used continue rather than break, so two sensors could each run a
 *         ~1 s begin() in a single iteration with no watchdog feed between.
 *       * the 3 s USB boot delay is skipped after a WDT reset and ends early
 *         on DTR. In flight there is no host, so that delay was pure added
 *         blackout on every in-flight reboot.
 *       * the debug line is one Serial.write instead of ~40 prints, each of
 *         which can stall ~70 ms on a host that is enumerated but not reading.
 *       * per-frame cosf/sinf replaced with exact tables (yaws are fixed
 *         multiples of 90 deg; the part has no FPU).
 *   - RAM, ~1.3 KB of 32 KB: MAVLINK_COMM_NUM_BUFFERS 1 drops ~900 B of unused
 *     per-channel statics; a shared send helper removes a 280-byte stack
 *     buffer and a copy from all five send paths; the ring is built in place;
 *     a bool[5][4] becomes a bitmask; duplicate name tables move to flash.
 *   - BUG FIX: STATUSTEXT is staged through a zero-filled buffer. The pack
 *     helper copies 50 bytes regardless of string length, so short literals
 *     were reading adjacent flash into the packet.
 *
 * NOT FIXED in v4.4, found while doing the above - decide these deliberately:
 *   - A sensor that produced data and was THEN latched off keeps
 *     lastGoodMs != 0, so the stale-data guard forces its whole FoV to MIN_CM
 *     for the rest of the session. The guard's comment only exempts sensors
 *     that NEVER produced data. RIGHT and LEFT both did exactly this on
 *     2026-09-07: with PRX1_TYPE enabled, AP would have seen permanent
 *     obstacles on both sides at once. Is that the intended fail-safe?
 *   - The latch is defeatable when one active sensor remains: mux-recovery
 *     clears reinitCycles, so begin() retries unbounded every ~500 ms - the
 *     bus-wedge exposure the latch exists to cap.
 *   - Wire.setClock is 1 MHz FM+. The dropouts appear in flight and not at
 *     boot, which reads as signal integrity. 400 kHz is a one-line experiment
 *     and a more direct test of the RIGHT/LEFT failures than anything above.
 *
 * v4.4 HAS NOT BEEN COMPILED. No Arduino toolchain was available where it was
 * edited. Build it before flashing.
 *
 * v4.1 changes over v4:
 *   - SENSOR ENABLE MASK: per-sensor compile-time disable. RIGHT is
 *     disabled by default — logs proved its begin() can wedge the shared
 *     I2C bus indefinitely (SAMD21 Wire has no transaction timeout), which
 *     blocks past the WDT window and reboots the whole board. Re-enable
 *     it in sensorEnabled[] once the module/cabling is replaced.
 *   - WDT BLACKBOX: reinit code arms a marker in .noinit RAM (survives a
 *     WDT reset, garbage after power-cycle) naming the sensor whose
 *     begin() is in flight. On boot, if the reset cause was the watchdog
 *     and the marker is valid, that sensor is auto-quarantined for the
 *     session ("ToF x quarantined (WDT)" STATUSTEXT). One reset per bad
 *     sensor per power cycle, instead of a reboot loop. Closes the v4 gap
 *     where a WDT reset wiped the backoff counters and restarted the
 *     2-reinits-then-fatal-3rd cycle forever.
 *   - Boot banner prints the firmware version so the running build is
 *     verifiable over USB.
 *
 * v4 changes over v3:
 *   - Reinit backoff: after REINIT_GIVEUP_CYCLES consecutive reinits with
 *     zero successful frames, a sensor is declared dead ("ToF x LOST"
 *     STATUSTEXT) and retried only every DEAD_RETRY_MS (30 s) instead of
 *     every ~5 s. Any successful frame resets the counter.
 *   - Watchdog widened 4 s -> 8 s.
 *   - SOLO_TEST_SENSOR build option for per-branch fault isolation.
 *
 * v3 changes over v2:
 *   - Reinit spread across loop iterations (max 1 per iteration).
 *   - Mux-recovery marks sensors for FAST reinit.
 *   - Adaptive attitude re-request (~1 s stale / 5 s healthy).
 *   - MAVLink STATUSTEXT on state transitions.
 *
 * Mux ports:
 *   0 -> front  (yaw   0)
 *   2 -> top    (pitch 90, upward-facing)
 *   3 -> right  (yaw  90)
 *   4 -> left   (yaw 270)
 *   7 -> back   (yaw 180)
 *
 * Output on Serial1:
 *   OBSTACLE_DISTANCE  - 72-bin 360 deg ring from 4 horizontal sensors
 *   DISTANCE_SENSOR    - minimum reading from top (upward) sensor
 *
 * ArduPilot receiving port:
 *   SERIALx_PROTOCOL = 2
 *   SERIALx_BAUD     = 230   (230400 bps)
 ****************************************************************/
#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <SparkFun_VL53L5CX_Library.h>
/* The MAVLink helpers keep a static mavlink_message_t (~290 B) + status per
   comm channel; the default is 4 channels. We only ever use MAVLINK_COMM_0,
   so declare that up front and get ~900 B of SRAM back (32 KB total). Must
   precede the include - mavlink_types.h only defaults it if unset. */
#define MAVLINK_COMM_NUM_BUFFERS 1
#include <MAVLink.h>
#include <math.h>

#define FW_VERSION "v4.4"

/* ---------------- user settings ---------------- */
#define NUM_SENS 5
const uint8_t muxPorts[NUM_SENS] = {3, 2, 0, 4, 7};
enum { S_RIGHT, S_TOP, S_FRONT, S_LEFT, S_BACK };

/* Per-sensor enable mask. A disabled sensor is never initialised, polled,
   or reinit'd; its ring bins stay MAV_NO_DATA (unknown) so AP simply has
   no information on that bearing (the stale-data guard does NOT force
   "obstacle" for a never-alive sensor).
   RIGHT is DISABLED: its begin() has been observed to wedge the shared
   I2C bus indefinitely, blowing through the watchdog and rebooting the
   board (blacking out all healthy sensors). Re-enable after the module
   or its cable/connector is replaced and bench-verified solo. */
static const bool sensorEnabled[NUM_SENS] = {
  true,   // S_RIGHT - kept enabled to match v4.2. NOTE the header above: its
          //           begin() has been seen to wedge the shared I2C bus and
          //           reboot the board, taking every healthy sensor with it.
          //           It went LOST in both 2026-09-07 flights.
  false,  // S_TOP   - not used for collision avoidance
  true,   // S_FRONT
  true,   // S_LEFT  - re-enabled to match v4.2 (reported OK in flight 2)
  true    // S_BACK
};

/* Solo-test mode: set to S_RIGHT / S_TOP / S_FRONT / S_LEFT / S_BACK to
   init and service ONLY that sensor (overrides sensorEnabled — the solo
   sensor runs even if disabled above, everything else is skipped). Use to
   test a suspect sensor with the shared rail at minimum load, isolating
   per-branch power/wiring faults from shared-supply problems.
   Set to -1 for normal operation. */
#define SOLO_TEST_SENSOR   (-1)

#define GRID_RES        16          // 4 x 4
#define COLS            4
#define SENSOR_HZ       60          // ToF internal ranging frequency
/* Per-bin refresh ceiling:
 *   - Datasheet: VL53L5CX caps 4x4 ranging at SENSOR_HZ = 60 Hz per sensor
 *     (sensors range in parallel, so this does NOT divide by NUM_SENS).
 *   - System: all 5 share one I2C bus through the mux; one getRangingData()
 *     transfer is several ms, so a full round-robin gives ~25 Hz refresh
 *     per bin at 1 MHz. Publishing faster just resends unchanged data, so
 *     20 Hz (a touch under the refresh ceiling) feeds AP's PRX1_FILT
 *     (target ~10 Hz) above Nyquist with room to spare. */
#define PUBLISH_HZ      20          // MAVLink publish rate
#define UART_BAUD       230400      // serial to ArduPilot (match SERIALx_BAUD=230).
                                    // 921600 was unreliable on Pixracer Pro SERIAL5
                                    // (UART7/debug port, no flow control); 230400 is
                                    // 2x the original 115200 with ample headroom at
                                    // 20 Hz publish.

#define MIN_CM          2          // VL53L5CX datasheet spec floor: 2 cm
#define MAX_CM          400
#define MAV_NO_DATA     UINT16_MAX

/* 72 bins around 360 degrees at 5 degrees per bin */
#define DEG_PER_BUCKET  5
#define NUM_BUCKETS     72

#define SYS_ID          10
#define COMP_ID         MAV_COMP_ID_OBSTACLE_AVOIDANCE

#define BEARING_SHIFT_DEG   0.0f
#define FRONT_BLIND_DEG     0.0f

/* debug over USB */
#define DEBUG_USB       1
#define DEBUG_USB_HZ    2

/* Per-zone dump for the 3D visualizer.
   Adds one "Z,<sensor_idx>,<d0>,<s0>,<d1>,<s1>,...,<d15>,<s15>" line per
   sensor at DEBUG_ZONES_HZ rate. Distance is mm, status is ST target_status.
   ~5 lines × ~120 chars/line/sec when enabled — negligible USB load. Set to
   0 to disable when not using the 3D visualizer. */
#define DEBUG_ZONES     0      // set to 1 to feed visualizer_3d.py
#define DEBUG_ZONES_HZ  5

/* sensor health recovery */
#define FAIL_REINIT_THRESHOLD   10
#define NOT_READY_TIMEOUT_MS    500

/* reinit backoff (v4 -> v4.3): after this many consecutive reinit cycles
   without a *sustained* recovery, the sensor is LATCHED off for the rest of
   the session (sensorQuarantined -> sensorExcluded skips it everywhere).
   Clears only on power cycle. This replaces v4's DEAD_RETRY_MS retry loop,
   which oscillated at the 30 s boundary — a flapping sensor (one frame per
   reinit, then dead) kept dodging the give-up and eventually wedged the bus
   into a WDT reset. Latching caps total reinit attempts at N, which caps
   bus-wedge exposure.
     A successful frame only clears the strike counter if the sensor has
   stayed alive for REINIT_PROBATION_MS since its last reinit (v4.2) — so a
   single post-reinit frame can't reset the strikes and dodge the latch. */
#define REINIT_GIVEUP_CYCLES    3
#define REINIT_PROBATION_MS     10000UL

/* WDT blackbox (v4.1): a marker in .noinit RAM survives a watchdog reset
   (only a power cycle scrambles it). reinit/init arm it with the sensor
   index before a blocking begin() and disarm afterwards. If the chip comes
   up from a WDT reset with the marker armed, that sensor's begin() is what
   hung the bus — quarantine it for the rest of the session. */
#define WDT_BB_MAGIC    0xB1ACB0C5UL

/* safety hardening */
#define SENSOR_STALE_MS         250   // a horizontal sensor silent longer than this is
                                      // forced to MIN_CM (assume obstacle) — fail safe
#define LONE_NEAR_CM            50    // a single valid zone closer than this is trusted as
                                      // a real obstacle instead of being suppressed
#define MUX_RETRY_MS            500   // how often loop() re-attempts mux recovery when down

/* Attitude-aware horizon projection.
   When ATTITUDE (msgid 30) is received from the FC on Serial1 RX, we
   know the drone's pitch/roll and can drop rows of each 4x4 frame that
   are looking too far above/below the horizon — i.e., the floor when
   pitched forward, or the ceiling when pitched back. Without this,
   ground returns during banked flight appear as phantom obstacles in
   the OBSTACLE_DISTANCE ring. */
#define ROWS                4
#define HORIZON_MASK_DEG    10.0f    // include zones within +/- this many deg of horizon
#define ATTITUDE_STALE_MS   1000     // ignore mask if attitude older than this

/* Row pitch in the sensor's body frame, in SPAD row order (p / COLS).
   SPAD bottom row (p=0..3) corresponds to scene TOP because of the
   lens's vertical flip, so spadRow 0 has the most positive pitch.
   Centre of each 11.25-deg row in the 45-deg vertical FoV. */
static const float rowScenePitchDeg[ROWS] = { +16.875f, +5.625f, -5.625f, -16.875f };

/* ---------------- state ---------------- */
QWIICMUX              mux;
SparkFun_VL53L5CX     tof[NUM_SENS];
VL53L5CX_ResultsData  frame;
uint16_t              sensorMin[NUM_SENS];
bool                  sensorOk[NUM_SENS];

/* per-sensor bucket contributions; the final 72-bin ring is built fresh
   each publish by taking the min across all sensors per bin, so overlapping
   FoVs no longer clobber each other */
uint16_t              sensorRing[NUM_SENS][COLS];

/* health tracking */
uint8_t               failCount[NUM_SENS];
uint32_t              lastReadyMs[NUM_SENS];

/* reinit backoff tracking (v4) */
uint8_t               reinitCycles[NUM_SENS] = {0};  // consecutive reinits w/o a good frame
uint32_t              lastReinitMs[NUM_SENS] = {0};

/* WDT blackbox (v4.1). .noinit: not zeroed by the startup code, so the
   contents survive a WDT/system reset. After a power-on reset they hold
   garbage — the magic word gates against acting on that. */
__attribute__((section(".noinit"))) volatile uint32_t wdt_bb_magic;
__attribute__((section(".noinit"))) volatile uint8_t  wdt_bb_sensor;

/* runtime quarantine flags, set at boot from the blackbox. Cleared only by
   a power cycle (deliberate: a sensor that hard-hung the bus once should
   not get another chance mid-session). */
bool                  sensorQuarantined[NUM_SENS] = { false };

/* cached mux port to skip redundant I2C writes */
static uint8_t        currentPort = 0xFF;

/* mux / bus health (safety) */
bool      muxOk        = false;
uint32_t  lastMuxTry   = 0;
uint32_t  lastGoodMs[NUM_SENS] = {0};   // last fully-successful read per sensor (0 = never)

/* "fast reinit needed" flag, set on mux-recovery so the per-sensor loop
   triggers reinit immediately instead of waiting for failCount warm-up.
   Spread one reinit per loop iteration so publishes interleave. */
bool      sensorNeedsReinit[NUM_SENS] = { false };

/* latest attitude received from the FC (radians; NED body-FRD frame:
   positive pitch = nose up, positive roll = right wing down) */
float    drone_roll_rad   = 0.0f;
float    drone_pitch_rad  = 0.0f;
uint32_t last_attitude_ms = 0;

/* RX diagnostic counters — exposed in the debug print so we can see
   exactly where the attitude pipeline is broken (no bytes? bytes but no
   parse? parse but no ATTITUDE?) */
uint32_t rx_bytes_total   = 0;   // raw bytes read from Serial1
uint32_t rx_msgs_total    = 0;   // MAVLink messages fully parsed
uint32_t rx_att_total     = 0;   // ATTITUDE (msgid 30) messages decoded

/* Throughput counters — incremented in loop()/publish block. The debug
   print converts deltas to Hz so we can verify PUBLISH_HZ is being hit
   and that nothing is stalling loop(). */
uint32_t pub_count_total  = 0;   // OBSTACLE_DISTANCE publish cycles
uint32_t loop_count_total = 0;   // main loop() iterations

/* last computed row mask per sensor (for the debug print), one bit per
   SPAD row: bit r set = row r used. Bit 0 = SPAD bottom row = SCENE TOP
   (high +pitch). Initialised to all-pass so the print shows "1111" until
   the first attitude arrives. */
uint8_t lastRowMask[NUM_SENS] = { 0x0F, 0x0F, 0x0F, 0x0F, 0x0F };

#if DEBUG_ZONES
/* Latest per-zone distance/status per sensor — for the 3D visualizer.
   Updated after each successful getRangingData(); dumped at DEBUG_ZONES_HZ. */
uint16_t lastZoneDist[NUM_SENS][GRID_RES];
uint8_t  lastZoneStat[NUM_SENS][GRID_RES];
uint32_t next_zone_dump_ms = 0;
/* Copy of the 72-bin ring exactly as last sent to AP (the ring itself is
   now built in place inside the outgoing OBSTACLE_DISTANCE packet). */
uint16_t ring[NUM_BUCKETS];
#endif

/* publish pacing */
const uint32_t PUBLISH_PERIOD_US = 1000000UL / PUBLISH_HZ;
uint32_t last_pub_us = 0;

/* heartbeat pacing */
uint32_t last_hb_ms = 0;
#define HEARTBEAT_MS 1000

/* debug pacing */
uint32_t next_dbg_ms = 0;

/* horizontal flag: top sensor is vertical, rest horizontal */
static const bool  horiz[NUM_SENS]  = { true, false, true, true, true };

/* yaw for horizontal sensors (top yaw unused, set to 0). Conventional CW
   BODY_FRD mapping, VERIFIED against AP's proximity GUI: an obstacle at the
   physical RIGHT sensor must appear at 90 deg (body right) in
   OBSTACLE_DISTANCE. (An earlier swap to 270/90 mirrored left<->right in the
   ring — front/back were unaffected, which is why only pitch avoidance
   worked. Must match SENSOR_YAW_DEG in the visualizers.) */
static const float yawDeg[NUM_SENS] = { 90, 0, 0, 270, 180 };

/* cos/sin of yawDeg[], used by the horizon row mask on every frame. The
   sensor yaws are fixed multiples of 90 deg, so these are exact and spare
   the M0+ (no FPU) a cosf()+sinf() pair per frame. MUST track yawDeg[]. */
static const float yawCos[NUM_SENS] = {  0.0f, 1.0f, 1.0f,  0.0f, -1.0f };
static const float yawSin[NUM_SENS] = {  1.0f, 0.0f, 0.0f, -1.0f,  0.0f };

/* sensor names for STATUSTEXT / USB banner (const pointers -> lives in flash) */
static const char* const sNames[NUM_SENS] = { "RIGHT","TOP","FRONT","LEFT","BACK" };

static const uint8_t orient[NUM_SENS] = {
  MAV_SENSOR_ROTATION_YAW_90,    // right (mux 3)
  MAV_SENSOR_ROTATION_PITCH_90,  // top (upward)
  MAV_SENSOR_ROTATION_NONE,      // front
  MAV_SENSOR_ROTATION_YAW_270,   // left (mux 4)
  MAV_SENSOR_ROTATION_YAW_180    // back
};

/* 4 column center bearings inside the 45 deg FoV.
   NOTE: the VL53L5CX's RX lens flips the captured image horizontally
   (datasheet section 5.1.3 "Effective zone orientation"). SPAD column 0
   (zone IDs 0, 4, 8, 12) is illuminated by targets on the RIGHT of the
   scene, so it must map to a POSITIVE yaw offset. This array is in SPAD
   column order (the loop indexes via p % COLS), so values are reversed
   relative to a "left-to-right scene" ordering. */
static const float colYaw4[COLS] = {+22.5f, +7.5f, -7.5f, -22.5f};

/* precomputed bucket index for each horizontal sensor and column */
uint8_t bucketIdx[NUM_SENS][COLS];

/* ---------------- helpers ---------------- */

/* true if sensor s must not be touched this session:
   - solo-test mode active and s isn't the solo sensor, or
   - (normal mode) s is compile-time disabled in sensorEnabled[], or
   - s was quarantined at boot by the WDT blackbox.
   In solo mode the solo sensor runs even if disabled/quarantined — solo
   exists precisely to bench-test suspect sensors. */
static inline bool sensorExcluded(uint8_t s)
{
#if SOLO_TEST_SENSOR >= 0
  return s != (uint8_t)SOLO_TEST_SENSOR;
#else
  return !sensorEnabled[s] || sensorQuarantined[s];
#endif
}

static inline void setMuxPort(uint8_t port)
{
  if (currentPort != port) {
    mux.setPort(port);
    currentPort = port;
  }
}

/* ---- safety: watchdog, I2C bus recovery, mux (re)init ---- */

/* Enable the SAMD21 hardware watchdog (~8 s), clocked from the always-on
   OSCULP32K via GCLK2 divided to ~1024 Hz. Once enabled, if loop() ever
   stops feeding it (hung I2C transaction, frozen loop), the chip resets
   and re-runs setup() — turning a silent freeze into auto-recovery.
   8 s covers a slow VL53L5CX begin() on a half-responsive sensor while
   still catching true bus hangs. A begin() that wedges outright is caught
   by the WDT blackbox: one reset, then the offending sensor is
   quarantined for the session. */
static void wdtEnable()
{
  GCLK->GENDIV.reg  = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(4);   // DIVSEL: 2^(4+1)=32 -> 1024 Hz
  while (GCLK->STATUS.bit.SYNCBUSY);
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_GENEN
                    | GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_DIVSEL;
  while (GCLK->STATUS.bit.SYNCBUSY);
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT | GCLK_CLKCTRL_CLKEN
                    | GCLK_CLKCTRL_GEN_GCLK2;
  while (GCLK->STATUS.bit.SYNCBUSY);

  WDT->CTRL.reg = 0;                       // disable while configuring
  while (WDT->STATUS.bit.SYNCBUSY);
  WDT->INTENCLR.reg = WDT_INTENCLR_EW;     // no early-warning interrupt
  WDT->CONFIG.reg   = WDT_CONFIG_PER_8K;   // 8192 cycles / 1024 Hz ~= 8 s
  WDT->CTRL.reg     = WDT_CTRL_ENABLE;
  while (WDT->STATUS.bit.SYNCBUSY);
}

static inline void wdtFeed()
{
  if (!WDT->STATUS.bit.SYNCBUSY) WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
}

/* Arm/disarm the WDT blackbox around any blocking begin(). If the WDT
   fires while armed, the next boot reads the sensor index and quarantines
   it. Disarm ASAP after begin() returns so an unrelated later hang isn't
   blamed on this sensor. */
static inline void wdtBlackboxArm(uint8_t s)
{
  wdt_bb_sensor = s;
  wdt_bb_magic  = WDT_BB_MAGIC;
}

static inline void wdtBlackboxDisarm()
{
  wdt_bb_magic = 0;
}

/* Release a slave that's holding the I2C bus low: bit-bang up to 9 SCL
   clocks until SDA goes high, then issue a STOP. Run before Wire owns the
   pins. This is what lets a power-glitch-induced bus hang recover instead
   of freezing mux.begin() forever. */
static void i2cBusRecover()
{
  pinMode(PIN_WIRE_SCL, OUTPUT);
  pinMode(PIN_WIRE_SDA, INPUT_PULLUP);
  for (uint8_t i = 0; i < 9; i++) {
    digitalWrite(PIN_WIRE_SCL, HIGH); delayMicroseconds(5);
    if (digitalRead(PIN_WIRE_SDA)) break;          // slave released SDA
    digitalWrite(PIN_WIRE_SCL, LOW);  delayMicroseconds(5);
  }
  /* STOP: SDA low->high while SCL high */
  pinMode(PIN_WIRE_SDA, OUTPUT);
  digitalWrite(PIN_WIRE_SDA, LOW);  delayMicroseconds(5);
  digitalWrite(PIN_WIRE_SCL, HIGH); delayMicroseconds(5);
  digitalWrite(PIN_WIRE_SDA, HIGH); delayMicroseconds(5);
}

/* (Re)initialise the mux: recover the bus, re-init Wire, re-begin the mux,
   invalidate the cached port. Used at boot and for mid-flight recovery if
   the mux browns out. Returns/sets muxOk. */
static bool beginMux()
{
  i2cBusRecover();
  Wire.begin();
  Wire.setClock(1000000UL);  // 1 MHz FM+ (drop to 400000UL for more noise immunity)
  currentPort = 0xFF;
  muxOk = mux.begin(0x70);
  return muxOk;
}

/* Single definition of "FC attitude is fresh": used by the STATUSTEXT edge
   detector and the adaptive stream re-request, which previously each
   computed it independently. */
static inline bool attitudeHealthy(uint32_t now_ms)
{
  return (last_attitude_ms != 0) && (now_ms - last_attitude_ms < ATTITUDE_STALE_MS);
}

/* Serialise a finalised message straight out of the mavlink_message_t onto
   Serial1, without staging it through a 280-byte mavlink_msg_to_send_buffer()
   copy on the stack (that was repeated in five send paths). Byte-for-byte
   what that helper produces: the finaliser has already trimmed msg.len (v2
   zero-trim) and written the two CRC bytes immediately after the payload
   inside payload64[] (mavlink_ck_a/b), so payload+CRC is one contiguous run.
   We never sign, so there is no signature block. */
static void mavlinkSend(const mavlink_message_t& msg)
{
  uint8_t hdr[MAVLINK_NUM_HEADER_BYTES];
  uint8_t hdr_len;
  hdr[0] = msg.magic;
  hdr[1] = msg.len;
  if (msg.magic == MAVLINK_STX_MAVLINK1) {
    hdr[2] = msg.seq;
    hdr[3] = msg.sysid;
    hdr[4] = msg.compid;
    hdr[5] = msg.msgid & 0xFF;
    hdr_len = MAVLINK_CORE_HEADER_MAVLINK1_LEN + 1;
  } else {
    hdr[2] = msg.incompat_flags;
    hdr[3] = msg.compat_flags;
    hdr[4] = msg.seq;
    hdr[5] = msg.sysid;
    hdr[6] = msg.compid;
    hdr[7] = msg.msgid & 0xFF;
    hdr[8] = (msg.msgid >> 8) & 0xFF;
    hdr[9] = (msg.msgid >> 16) & 0xFF;
    hdr_len = MAVLINK_NUM_HEADER_BYTES;
  }
  Serial1.write(hdr, hdr_len);
  Serial1.write((const uint8_t*)_MAV_PAYLOAD(&msg),
                (size_t)msg.len + MAVLINK_NUM_CHECKSUM_BYTES);
}

static inline void sensor_ring_clear()
{
  for (uint8_t s = 0; s < NUM_SENS; s++)
    for (uint8_t c = 0; c < COLS; c++)
      sensorRing[s][c] = MAV_NO_DATA;
}

static inline void sensor_ring_clear_one(uint8_t s)
{
  for (uint8_t c = 0; c < COLS; c++)
    sensorRing[s][c] = MAV_NO_DATA;
}

static inline void sensor_min_clear()
{
  for (uint8_t i = 0; i < NUM_SENS; i++)
    sensorMin[i] = MAV_NO_DATA;
}

/* Drain any inbound MAVLink bytes on Serial1 (we listen for ATTITUDE from
   the FC). Non-blocking; just consumes whatever is in the RX buffer.
   Bumps diagnostic counters so we can see the pipeline status. */
static inline void readIncomingMavlink()
{
  static mavlink_message_t rx_msg;
  static mavlink_status_t  rx_status;
  while (Serial1.available()) {
    uint8_t c = Serial1.read();
    rx_bytes_total++;
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &rx_msg, &rx_status)) {
      rx_msgs_total++;
      if (rx_msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
        mavlink_attitude_t att;
        mavlink_msg_attitude_decode(&rx_msg, &att);
        drone_roll_rad   = att.roll;
        drone_pitch_rad  = att.pitch;
        last_attitude_ms = millis();
        rx_att_total++;
      }
    }
  }
}

static inline void sendObstacleDistance()
{
  /* Build the 72-bin ring fresh from per-sensor contributions. Each
     column's 11.25-deg FoV is spread across the centre bucket and the
     two adjacent buckets (covers ~15 deg, slightly wider than the FoV,
     which is fine and matches the column's actual angular extent). This
     fills 48 of 72 bins instead of 16, removing aliasing-style holes
     between adjacent columns within a sensor. The remaining 24 ND bins
     are in the genuine inter-sensor gaps (e.g., +22.5° to +67.5°
     between FRONT's right edge and LEFT's left edge). */
  mavlink_obstacle_distance_t od = {};
  uint16_t* const ring = od.distances;   // build the ring in the packet itself

  const uint32_t now_ms = millis();
  for (uint8_t i = 0; i < NUM_BUCKETS; i++) ring[i] = MAV_NO_DATA;
  for (uint8_t s = 0; s < NUM_SENS; s++) {
    if (!horiz[s]) continue;
    /* Stale-data guard: if a sensor was producing data but has gone silent
       longer than SENSOR_STALE_MS, force its whole FoV to MIN_CM (assume
       obstacle) rather than letting AP act on the last stale reading. A
       sensor that never produced data (lastGoodMs==0, e.g. failed init or
       excluded/quarantined) is left as-is (its bins are MAV_NO_DATA =
       unknown), so a permanently absent sensor doesn't permanently block
       that bearing. */
    bool stale = (lastGoodMs[s] != 0) && (now_ms - lastGoodMs[s] > SENSOR_STALE_MS);
    for (uint8_t c = 0; c < COLS; c++) {
      uint16_t v = stale ? (uint16_t)MIN_CM : sensorRing[s][c];
      if (v == MAV_NO_DATA) continue;
      uint8_t center = bucketIdx[s][c];
      for (int8_t off = -1; off <= 1; off++) {
        uint8_t b = uint8_t((center + off + NUM_BUCKETS) % NUM_BUCKETS);
        if (ring[b] == MAV_NO_DATA || v < ring[b])
          ring[b] = v;
      }
    }
  }

  od.time_usec    = micros();
  od.frame        = MAV_FRAME_BODY_FRD;
  od.sensor_type  = MAV_DISTANCE_SENSOR_LASER;
  od.increment    = DEG_PER_BUCKET;
  od.angle_offset = 0;
  od.min_distance = MIN_CM;
  od.max_distance = MAX_CM;

#if DEBUG_ZONES
  memcpy(::ring, od.distances, sizeof(::ring));   // snapshot for debug_zones_dump()
#endif

  mavlink_message_t msg;
  mavlink_msg_obstacle_distance_encode(SYS_ID, COMP_ID, &msg, &od);
  mavlinkSend(msg);
}

static inline void sendDistanceSensorTop()
{
  if (sensorMin[S_TOP] == MAV_NO_DATA) return;

  mavlink_distance_sensor_t ds = {};
  ds.time_boot_ms     = millis();
  ds.min_distance      = MIN_CM;
  ds.max_distance      = MAX_CM;
  ds.current_distance  = sensorMin[S_TOP];
  ds.type              = MAV_DISTANCE_SENSOR_LASER;
  ds.id                = S_TOP;
  ds.orientation       = MAV_SENSOR_ROTATION_PITCH_90;
  ds.covariance        = 0;          // unknown
  ds.horizontal_fov    = 0.7854f;   // ~45 deg
  ds.vertical_fov      = 0.7854f;
  ds.signal_quality    = 100;       // valid reading

  mavlink_message_t msg;
  mavlink_msg_distance_sensor_encode(SYS_ID, COMP_ID, &msg, &ds);
  mavlinkSend(msg);
}

/* Send a MAVLink STATUSTEXT (msgid 253). Visible in Mission Planner's
   Messages tab + dataflash log; perfect for transition-only status notes. */
static inline void sendStatusText(uint8_t severity, const char* text)
{
  /* The pack helper copies a fixed 50 bytes from `text` regardless of its
     real length, so a short literal used to drag whatever followed it in
     flash into the packet. Stage it through a zero-filled 50-char buffer:
     the decoded text is unchanged, the read overrun is gone, and MAVLink 2's
     trailing-zero trim now actually shortens the packet on the wire. */
  char padded[50] = { 0 };
  strncpy(padded, text, sizeof(padded));
  mavlink_message_t msg;
  mavlink_msg_statustext_pack(SYS_ID, COMP_ID, &msg, severity, padded, 0, 0);
  mavlinkSend(msg);
}

/* Watch for state transitions and emit a STATUSTEXT on each edge only.
   The first invocation syncs internal "previous" state to current state so
   no boot-time spam — only real changes after that fire. */
static void checkStatusTransitions()
{
  static bool initialised = false;
  static bool prev_mux  = false;
  static bool prev_sens[NUM_SENS] = { false, false, false, false, false };
  static bool prev_att  = false;

  bool att_healthy = attitudeHealthy(millis());

  if (!initialised) {
    prev_mux = muxOk;
    for (uint8_t i = 0; i < NUM_SENS; i++) prev_sens[i] = sensorOk[i];
    prev_att = att_healthy;
    initialised = true;
    return;
  }

  /* mux edges */
  if (muxOk != prev_mux) {
    sendStatusText(muxOk ? MAV_SEVERITY_INFO : MAV_SEVERITY_CRITICAL,
                   muxOk ? "TCA9548A mux OK" : "TCA9548A mux LOST");
    prev_mux = muxOk;
  }

  /* per-sensor edges */
  for (uint8_t i = 0; i < NUM_SENS; i++) {
    if (sensorOk[i] != prev_sens[i]) {
      char buf[40];
      snprintf(buf, sizeof(buf), "ToF %s %s",
               sNames[i], sensorOk[i] ? "OK" : "LOST");
      sendStatusText(sensorOk[i] ? MAV_SEVERITY_INFO : MAV_SEVERITY_WARNING,
                     buf);
      prev_sens[i] = sensorOk[i];
    }
  }

  /* attitude edges */
  if (att_healthy != prev_att) {
    sendStatusText(att_healthy ? MAV_SEVERITY_INFO : MAV_SEVERITY_WARNING,
                   att_healthy ? "FC attitude OK" : "FC attitude LOST");
    prev_att = att_healthy;
  }
}

static inline void sendHeartbeat()
{
  uint32_t now = millis();
  if (now - last_hb_ms >= HEARTBEAT_MS) {
    last_hb_ms = now;

    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(SYS_ID, COMP_ID, &msg,
      MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID,
      0, 0, MAV_STATE_ACTIVE);
    mavlinkSend(msg);
  }
}

/* Ask the FC to start streaming ATTITUDE at the desired rate. This is the
   modern, peer-explicit way (MAV_CMD_SET_MESSAGE_INTERVAL) and bypasses
   the SRn_EXTRA1 stream-rate mechanism — useful when AP doesn't classify
   us as a GCS and so doesn't auto-activate the EXTRA1 stream. We re-send
   periodically in case the FC reboots. */
#define REQUEST_PERIOD_MS    5000
#define ATTITUDE_INTERVAL_US 50000   // 50000 us = 20 Hz
uint32_t last_request_ms = 0;

static inline void requestAttitudeStream()
{
  uint32_t now = millis();
  /* Adaptive re-request: hammer at 1 Hz while attitude is missing/stale so
     an FC reboot recovers fast; ease off to REQUEST_PERIOD_MS (5 s) once
     it's flowing again to avoid bus chatter. */
  uint32_t period = attitudeHealthy(now) ? REQUEST_PERIOD_MS : 1000UL;
  if (now - last_request_ms < period) return;
  last_request_ms = now;

  mavlink_message_t msg;
  mavlink_msg_command_long_pack(SYS_ID, COMP_ID, &msg,
    /* target_system    */ 1,    // ArduPilot default
    /* target_component */ 1,    // MAV_COMP_ID_AUTOPILOT1
    MAV_CMD_SET_MESSAGE_INTERVAL,
    /* confirmation     */ 0,
    /* param1: msg id   */ (float)MAVLINK_MSG_ID_ATTITUDE,
    /* param2: interval */ (float)ATTITUDE_INTERVAL_US,
    0, 0, 0, 0, 0);
  mavlinkSend(msg);
}

/* Full VL53L5CX bring-up: select its mux port, begin() (firmware upload,
   ~1 s of blocking I2C), then the ranging configuration. ONE copy, shared by
   boot-time init and mid-session reinit so the two sequences can't drift.
   Sets sensorOk[s] from the result.

   v4.3: blackbox armed across the ENTIRE I2C sequence, not just begin().
   v4.1 disarmed right after begin() returned, but the bus can wedge in the
   setResolution/startRanging calls that follow — those hangs escaped the
   armed window, so the WDT reset booted with no sensor named and failed to
   quarantine the culprit (seen in the v4.2 logs). Disarm only after the
   whole config sequence completes.

   Datasheet/UM2884 best-practice for obstacle avoidance:
   - target order CLOSEST: when a zone has multiple returns (e.g., glass
     in front of wall), report the nearest one, not strongest. Default is
     strongest, which biases toward big background objects.
   - sharpener ~20: edge enhancement so a near object in the FoV isn't
     blurred into its far neighbours. Default is 5 (mild). */
static bool bringUpSensor(uint8_t s)
{
  setMuxPort(muxPorts[s]);
  wdtBlackboxArm(s);
  bool ok = tof[s].begin();
  if (ok) {
    tof[s].setResolution(GRID_RES);
    tof[s].setRangingFrequency(SENSOR_HZ);
    tof[s].setTargetOrder(SF_VL53L5CX_TARGET_ORDER::CLOSEST);
    tof[s].setSharpenerPercent(20);
    tof[s].startRanging();
  }
  sensorOk[s] = ok;
  wdtBlackboxDisarm();
  return ok;
}

static void reinitSensor(uint8_t s)
{
  /* v4: count consecutive reinit attempts for the backoff gate. Reset to 0
     only by a successful frame (or mux recovery), so a sensor that never
     produces data accumulates cycles and eventually goes LOST. */
  if (reinitCycles[s] < 255) reinitCycles[s]++;
  lastReinitMs[s] = millis();

#if DEBUG_USB
  Serial.print(F("Reinit sensor "));
  Serial.print(s);
  Serial.print(F("... "));
#endif
  bool ok = bringUpSensor(s);
#if DEBUG_USB
  Serial.println(ok ? F("OK") : F("FAIL"));
#endif
  failCount[s] = 0;
  lastReadyMs[s] = millis();
  sensor_ring_clear_one(s);
  sensorMin[s] = MAV_NO_DATA;
}

/* Backoff gate around reinitSensor() (v4.3). A sensor that has burned
   REINIT_GIVEUP_CYCLES reinits without a sustained recovery is LATCHED off
   for the session: sensorQuarantined is set, so sensorExcluded() skips it
   from every path (service loop, mux-recovery marking, whole-bus check) from
   here on. sensorOk going false fires one "ToF x LOST" STATUSTEXT via
   checkStatusTransitions(). Clears only on power cycle.
     Unlike v4's version this cannot oscillate: a quarantined sensor is never
   serviced again, so it never re-enters this gate. reinitCycles is cleared
   (and the latch avoided) only by REINIT_PROBATION_MS of sustained uptime —
   see the getRangingData success path in loop(). */
static void maybeReinit(uint8_t s)
{
  if (reinitCycles[s] >= REINIT_GIVEUP_CYCLES) {
    sensorQuarantined[s] = true;   // latch off for the session
    sensorOk[s]          = false;  // -> "ToF x LOST" edge
    return;
  }
  reinitSensor(s);
}

#if DEBUG_ZONES
/* Emit per-sensor zone data plus the composited 72-bin ring that was
   just sent to AP. Format:
     Z,<sensor_idx>,<d0_mm>,<s0>,<d1_mm>,<s1>,...,<d15_mm>,<s15>   (5 lines)
     R,<b0>,<b1>,...,<b71>                                           (1 line)
   Distances in mm for Z, cm for R (matches the on-wire OBSTACLE_DISTANCE).
   Parsed by visualizer_3d.py to draw per-zone rays + ring outline. */
static inline void debug_zones_dump()
{
  const uint32_t now = millis();
  if ((int32_t)(now - next_zone_dump_ms) < 0) return;
  next_zone_dump_ms = now + (1000U / DEBUG_ZONES_HZ);

  for (uint8_t s = 0; s < NUM_SENS; s++) {
    Serial.print(F("Z,"));
    Serial.print(s);
    for (uint8_t p = 0; p < GRID_RES; p++) {
      Serial.print(F(","));
      Serial.print(lastZoneDist[s][p]);
      Serial.print(F(","));
      Serial.print(lastZoneStat[s][p]);
    }
    Serial.println();
  }

  /* ring[] was just (re)built by sendObstacleDistance() in this same
     publish cycle, so it reflects exactly what AP just received. */
  Serial.print(F("R"));
  for (uint8_t i = 0; i < NUM_BUCKETS; i++) {
    Serial.print(F(","));
    Serial.print(ring[i]);
  }
  Serial.println();
}
#endif

#if DEBUG_USB
/* Debug-print helpers. The whole status line is formatted into one buffer
   and handed to the USB CDC in a single write: with a host enumerated but
   not draining the port, every CDC write can stall up to its TX timeout
   (~70 ms on the SAMD core), and the old line was ~40 separate prints. */
static void dbgSensorVal(uint8_t s, char* out)   /* out: >= 6 bytes */
{
  if (sensorExcluded(s)) { strcpy(out, "OFF"); return; }   // disabled/quarantined
  uint16_t d = sensorMin[s];
  if (d == MAV_NO_DATA)   strcpy(out, "ND");               // sensor failed / no info
  else if (d > MAX_CM)    strcpy(out, "CLR");              // read OK, nothing in range
  else                    snprintf(out, 6, "%u", (unsigned)d);   // actual cm reading
}

/* mask bits print as 4 chars: leftmost = SPAD row 0 = SCENE TOP (high +pitch),
   rightmost = SPAD row 3 = SCENE BOTTOM (high -pitch). 1=row used, 0=masked. */
static void dbgRowMask(uint8_t s, char* out)     /* out: >= ROWS+1 bytes */
{
  if (!horiz[s]) { strcpy(out, "----"); return; }
  for (uint8_t r = 0; r < ROWS; r++)
    out[r] = (lastRowMask[s] & (1u << r)) ? '1' : '0';
  out[ROWS] = '\0';
}
#endif

static inline void debug_print_line()
{
#if DEBUG_USB
  const uint32_t now = millis();
  if ((int32_t)(now - next_dbg_ms) < 0) return;

  /* Compute throughput rates over the interval since the last debug print.
     The first call has dt=0 so we just seed the counters. Integer math:
     pub rate in tenths of Hz, loop rate in whole Hz (no float printf, which
     newlib-nano doesn't link by default). */
  static uint32_t last_dbg_ms       = 0;
  static uint32_t last_pub_count    = 0;
  static uint32_t last_loop_count   = 0;
  uint32_t pub_hz10 = 0;
  uint32_t loop_hz  = 0;
  if (last_dbg_ms != 0) {
    uint32_t dt_ms = now - last_dbg_ms;
    if (dt_ms > 0) {
      pub_hz10 = (pub_count_total  - last_pub_count)  * 10000UL / dt_ms;
      loop_hz  = (loop_count_total - last_loop_count) * 1000UL  / dt_ms;
    }
  }
  last_dbg_ms      = now;
  last_pub_count   = pub_count_total;
  last_loop_count  = loop_count_total;

  next_dbg_ms = now + (1000U / DEBUG_USB_HZ);

  /* --- attitude prefix --- */
  char att[32];
  if (last_attitude_ms == 0) {
    strcpy(att, "ATT:NONE");                     // never received from FC
  } else {
    uint32_t age = now - last_attitude_ms;
    if (age > ATTITUDE_STALE_MS) {
      snprintf(att, sizeof(att), "ATT:STALE(%lums)", (unsigned long)age);
    } else {
      snprintf(att, sizeof(att), "r=%d p=%d(%lums)",
               int(drone_roll_rad  * (180.0f / PI)),
               int(drone_pitch_rad * (180.0f / PI)),
               (unsigned long)age);
    }
  }

  /* --- per-sensor min readings + row mask --- */
  char vF[6], vR[6], vB[6], vL[6], vT[6];
  char mF[ROWS + 1], mR[ROWS + 1], mB[ROWS + 1], mL[ROWS + 1];
  dbgSensorVal(S_FRONT, vF); dbgRowMask(S_FRONT, mF);
  dbgSensorVal(S_RIGHT, vR); dbgRowMask(S_RIGHT, mR);
  dbgSensorVal(S_BACK,  vB); dbgRowMask(S_BACK,  mB);
  dbgSensorVal(S_LEFT,  vL); dbgRowMask(S_LEFT,  mL);
  dbgSensorVal(S_TOP,   vT);

  char line[192];
  int n = snprintf(line, sizeof(line),
    "%s F:%s/%s R:%s/%s B:%s/%s L:%s/%s T:%s cm  [pub=%lu.%luHz loop=%luHz"
    " | RX b=%lu m=%lu att=%lu]\r\n",
    att, vF, mF, vR, mR, vB, mB, vL, mL, vT,
    (unsigned long)(pub_hz10 / 10), (unsigned long)(pub_hz10 % 10),
    (unsigned long)loop_hz,
    (unsigned long)rx_bytes_total, (unsigned long)rx_msgs_total,
    (unsigned long)rx_att_total);
  if (n > 0) {
    if ((size_t)n >= sizeof(line)) n = sizeof(line) - 1;   // truncated: send what fits
    Serial.write((const uint8_t*)line, (size_t)n);
  }
#endif
}

/* ---------------- setup ---------------- */
void setup()
{
  /* Read the WDT blackbox FIRST, before anything can disturb it. If the
     last reset was a watchdog reset and the blackbox was armed, the named
     sensor's begin() is what hung — quarantine it for this session.
     RCAUSE is set by hardware at reset and stable to read any time. */
  bool wdtReset = (PM->RCAUSE.reg & PM_RCAUSE_WDT) != 0;
  uint8_t quarantinedNow = 0xFF;
  if (wdtReset && wdt_bb_magic == WDT_BB_MAGIC && wdt_bb_sensor < NUM_SENS) {
    sensorQuarantined[wdt_bb_sensor] = true;
    quarantinedNow = wdt_bb_sensor;
  }
  wdt_bb_magic = 0;   // consume the blackbox either way

#if DEBUG_USB
  Serial.begin(115200);
  /* Give USB CDC time to enumerate so the boot banner is visible - but only
     on a cold/external reset. After a WATCHDOG reset every sensor is already
     dark and AP is flying blind until loop() publishes again; a fixed 3 s
     wait for a monitor that may not be attached just stretched that
     blackout (it ran on every one of the in-flight reboots). A monitor that
     is already open (DTR asserted) needs no wait at all, so the banner still
     shows on the bench. */
  if (!wdtReset) {
    const uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 3000) { }
  }
  Serial.println(F("QT Py 5x VL53L5CX booting (" FW_VERSION ")"));
  if (wdtReset) Serial.println(F("*** reset cause: WATCHDOG ***"));
  if (quarantinedNow != 0xFF) {
    Serial.print(F("*** ToF "));
    Serial.print(sNames[quarantinedNow]);
    Serial.println(F(" QUARANTINED: its begin() hung the bus (WDT blackbox) ***"));
  }
  for (uint8_t i = 0; i < NUM_SENS; i++) {
    if (!sensorEnabled[i]) {
      Serial.print(F("*** ToF "));
      Serial.print(sNames[i]);
      Serial.println(F(" DISABLED in sensorEnabled[] ***"));
    }
  }
#if SOLO_TEST_SENSOR >= 0
  Serial.print(F("*** SOLO TEST MODE: sensor "));
  Serial.print(SOLO_TEST_SENSOR);
  Serial.println(F(" only ***"));
#endif
  Serial.flush();
#endif

  /* Enable the watchdog now, before any I2C activity. From here on, a hung
     Wire transaction (stuck bus) auto-resets the chip instead of freezing
     forever. The bounded USB wait above ran unguarded on purpose (it's a
     fixed-ceiling delay, can't hang). */
  wdtEnable();

  Serial1.begin(UART_BAUD);

  /* Force MAVLink v2 framing on COMM_0. OBSTACLE_DISTANCE (msgid 330) is
     v2-only; some library versions default the out flag to v1 and silently
     drop high-msgid messages. This is a no-op on v2-default libraries. */
  mavlink_status_t* chan_status = mavlink_get_channel_status(MAVLINK_COMM_0);
  chan_status->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;

  /* Bring up the mux with bus recovery + bounded retry. Never hang: if the
     mux can't be found, proceed anyway — loop() keeps retrying recovery,
     and the stale-data guard reports obstacles (safe) until it's back. */
  for (uint8_t attempt = 0; attempt < 20 && !muxOk; attempt++) {
    wdtFeed();
    if (beginMux()) break;
    delay(100);
  }
#if DEBUG_USB
  Serial.println(muxOk ? F("Mux OK") : F("Mux NOT found - will retry in loop()"));
#endif

  for (uint8_t i = 0; muxOk && i < NUM_SENS; i++) {
    wdtFeed();                 // VL53L5CX begin() uploads firmware (~1 s) - feed between
    sensorOk[i]  = false;
    failCount[i] = 0;
    if (sensorExcluded(i)) {
#if DEBUG_USB
      Serial.print(F("Init "));
      Serial.print(sNames[i]);
      Serial.println(F("... SKIP (excluded)"));
#endif
      continue;
    }
#if DEBUG_USB
    Serial.print(F("Init "));
    Serial.print(sNames[i]);
    Serial.print(F(" (mux "));
    Serial.print(muxPorts[i]);
    Serial.print(F(")... "));
#endif
    /* Blackbox armed across the WHOLE boot-time I2C sequence (begin +
       config) inside bringUpSensor(). If any call wedges the bus here, the
       reboot quarantines the culprit and boot completes next time instead
       of looping. */
    bool ok = bringUpSensor(i);
#if DEBUG_USB
    Serial.println(ok ? F("OK") : F("FAIL"));
#else
    (void)ok;
#endif
  }

  /* precompute bucket indices for horizontal sensors only */
  for (uint8_t s = 0; s < NUM_SENS; s++) {
    if (!horiz[s]) continue;
    for (uint8_t c = 0; c < COLS; c++) {
      float bearing = yawDeg[s] + colYaw4[c] + BEARING_SHIFT_DEG;
      /* bucket 0 = 0 deg (forward), wraps around 360 */
      float bmod = fmodf(bearing + 360.0f, 360.0f);
      bucketIdx[s][c] = uint8_t(lroundf(bmod / DEG_PER_BUCKET)) % NUM_BUCKETS;
    }
  }

  sensor_ring_clear();
  sensor_min_clear();
  for (uint8_t s = 0; s < NUM_SENS; s++) lastReadyMs[s] = millis();
  last_pub_us = micros();
  next_dbg_ms = millis();

  /* One-shot boot summary on the MAVLink link. Mission Planner shows it in
     the Messages tab so the operator sees how many sensors came up. */
  {
    uint8_t okCount = 0, exclCount = 0;
    for (uint8_t i = 0; i < NUM_SENS; i++) {
      if (sensorOk[i]) okCount++;
      if (sensorExcluded(i)) exclCount++;
    }
    char buf[48];
    snprintf(buf, sizeof(buf), "tof5x " FW_VERSION ": %u/%u OK, %u off%s",
             okCount, NUM_SENS, exclCount, muxOk ? "" : " (mux LOST)");
    sendStatusText(muxOk && okCount == (uint8_t)(NUM_SENS - exclCount)
                    ? MAV_SEVERITY_INFO
                    : MAV_SEVERITY_WARNING, buf);
    if (quarantinedNow != 0xFF) {
      char qbuf[48];
      snprintf(qbuf, sizeof(qbuf), "ToF %s quarantined (WDT)",
               sNames[quarantinedNow]);
      sendStatusText(MAV_SEVERITY_CRITICAL, qbuf);
    }
  }
}

/* round-robin sensor start index (advances by 1 per loop so polling order
   doesn't pin to one sensor) */
uint8_t nextSens = 0;

/* ---------------- loop ---------------- */
void loop()
{
  loop_count_total++;
  wdtFeed();   // pet the watchdog every iteration

  /* drain inbound MAVLink (ATTITUDE) before anything else, so the row
     mask we apply this iteration uses the freshest attitude available */
  readIncomingMavlink();

  if (!muxOk) {
    /* Bus/mux is down. Don't touch sensors. Periodically run recovery; on
       success, MARK every sensor for fast reinit — the actual begin() runs
       one per loop iteration below, so publishes interleave instead of
       blacking out for ~5 s. The stale-data guard forces "obstacle" for
       any sensor still ND so AP holds position rather than flying blind. */
    if (millis() - lastMuxTry > MUX_RETRY_MS) {
      lastMuxTry = millis();
      if (beginMux()) {
        for (uint8_t k = 0; k < NUM_SENS; k++) {
          sensorOk[k]          = false;
          sensorNeedsReinit[k] = !sensorExcluded(k);
          reinitCycles[k]      = 0;   // v4: fresh chances after a bus event
        }
      }
    }
  } else {
    /* service one ready sensor this iteration (round-robin) */
    uint8_t s = nextSens;
    for (uint8_t tried = 0; tried < NUM_SENS; tried++) {
      uint8_t idx = s;
      s = (s + 1) % NUM_SENS;

      if (sensorExcluded(idx)) continue;   // disabled/quarantined/solo-skipped

      if (!sensorOk[idx]) {
        /* Fast path: mux just recovered and marked this sensor for reinit.
           Skip the failCount warm-up and reinit now. ONLY ONE per loop
           iteration so the publish gate keeps firing between begins.
           (Direct reinit, no backoff gate: mux recovery reset the cycle
           counters, and a fresh bus deserves an immediate attempt.) */
        if (sensorNeedsReinit[idx]) {
          reinitSensor(idx);
          sensorNeedsReinit[idx] = false;
          break;
        }
        /* Slow path: mid-flight failure. Ramp up failCount, then reinit
           when threshold reached — still one reinit max per iteration,
           and gated by the v4 backoff once the sensor is declared dead. */
        if (millis() - lastReadyMs[idx] > NOT_READY_TIMEOUT_MS) {
          failCount[idx]++;
          lastReadyMs[idx] = millis();
          if (failCount[idx] >= FAIL_REINIT_THRESHOLD) {
            maybeReinit(idx);
            break;
          }
        }
        continue;
      }

      setMuxPort(muxPorts[idx]);

      if (!tof[idx].isDataReady()) {
        if (millis() - lastReadyMs[idx] > NOT_READY_TIMEOUT_MS) {
          failCount[idx]++;
          lastReadyMs[idx] = millis();
#if DEBUG_USB
          Serial.print(F("Sensor "));
          Serial.print(idx);
          Serial.print(F(" not-ready ("));
          Serial.print(failCount[idx]);
          Serial.println(F(")"));
#endif
          if (failCount[idx] >= FAIL_REINIT_THRESHOLD) {
            maybeReinit(idx);
            break;   // at most ONE reinit (~1 s of blocking I2C) per iteration
          }
        }
        continue;
      }

      lastReadyMs[idx] = millis();

      if (!tof[idx].getRangingData(&frame)) {
        failCount[idx]++;
#if DEBUG_USB
        Serial.print(F("Sensor "));
        Serial.print(idx);
        Serial.print(F(" getRangingData fail ("));
        Serial.print(failCount[idx]);
        Serial.println(F(")"));
#endif
        if (failCount[idx] >= FAIL_REINIT_THRESHOLD) {
          maybeReinit(idx);
          break;   // at most ONE reinit (~1 s of blocking I2C) per iteration
        }
        continue;
      }

      failCount[idx] = 0;
      /* v4.2/4.3: clear the reinit strike counter ONLY after the sensor has
         stayed alive for the full probation window since its last reinit. A
         flapping sensor that yields one frame per reinit never survives
         probation, so its strikes accumulate to the latch instead of being
         reset by that lone frame. reinitCycles==0 means it was never in
         recovery, so nothing to clear. */
      if (reinitCycles[idx] != 0 &&
          millis() - lastReinitMs[idx] > REINIT_PROBATION_MS) {
        reinitCycles[idx] = 0;
      }

#if DEBUG_ZONES
      /* Snapshot raw zones for the 3D visualizer. We do this BEFORE the
         filter cascade so the host sees the sensor's actual outputs;
         it'll apply its own validity logic. */
      for (uint8_t p = 0; p < GRID_RES; p++) {
        lastZoneDist[idx][p] = frame.distance_mm[p];
        lastZoneStat[idx][p] = frame.target_status[p];
      }
#endif

      /* Attitude-aware row mask: for horizontal sensors only, compute
         which rows are looking near the horizon vs at the floor/ceiling.
         Sensor's effective tilt from horizon, given drone attitude:
            pitch_eff = drone_pitch * cos(yaw_s) - drone_roll * sin(yaw_s)
         Then each row's world pitch = pitch_eff + row's body-frame pitch.
         If attitude is stale or this is the top sensor, all rows pass. */
      uint8_t rowMask = 0x0F;   // bit r set = SPAD row r passes
      if (horiz[idx] && (millis() - last_attitude_ms) < ATTITUDE_STALE_MS) {
        float pitch_eff_deg = (drone_pitch_rad * yawCos[idx]
                             - drone_roll_rad  * yawSin[idx]) * (180.0f / PI);
        rowMask = 0;
        for (uint8_t r = 0; r < ROWS; r++) {
          float world_pitch = pitch_eff_deg + rowScenePitchDeg[r];
          if (fabsf(world_pitch) <= HORIZON_MASK_DEG) rowMask |= (uint8_t)(1u << r);
        }
      }
      lastRowMask[idx] = rowMask;

      uint16_t colMin[COLS] = { MAV_NO_DATA, MAV_NO_DATA, MAV_NO_DATA, MAV_NO_DATA };
      uint8_t  colCnt[COLS] = {0};
      uint16_t smin         = MAV_NO_DATA;

      for (uint8_t p = 0; p < GRID_RES; p++) {
        /* ST validity gate: target_status 5 = range valid; 9 = range valid
           with reduced confidence. Everything else (wrap-around, low signal,
           hardware fault, no target) is unreliable garbage that shows up as
           phantom obstacles if accepted. */
        uint8_t st = frame.target_status[p];
        if (st != 5 && st != 9) continue;

        /* horizon row mask (no-op when sensor is top or attitude is stale) */
        if (!(rowMask & (1u << (p / COLS)))) continue;

        uint16_t d_cm = frame.distance_mm[p] / 10;
        if (d_cm > MAX_CM) continue;            // beyond range: drop
        if (d_cm < MIN_CM) d_cm = MIN_CM;       // closer than spec floor: clamp.
                                                 // The sensor's status was already
                                                 // valid (5 or 9), so something IS
                                                 // there — saturating at MIN_CM
                                                 // gives AP the strongest "very
                                                 // close obstacle" signal instead
                                                 // of falsely publishing CLEAR.

        uint8_t c = p % COLS;
        if (colMin[c] == MAV_NO_DATA || d_cm < colMin[c]) colMin[c] = d_cm;
        colCnt[c]++;
        if (smin == MAV_NO_DATA || d_cm < smin) smin = d_cm;
      }

      /* After a successful getRangingData(), "no in-range hit" means
         "clear", not "no data". MAVLink uses max_distance+1 as the
         "no obstacle present" sentinel; leaving values at MAV_NO_DATA
         makes ArduPilot hold the previous reading instead of clearing
         the bin. */
      if (smin == MAV_NO_DATA) smin = MAX_CM + 1;
      sensorMin[idx] = smin;
      lastGoodMs[idx] = millis();   // stamp for the stale-data guard

      /* Aggregate per column: MIN of valid zones.
         - >=2 valid zones: trust the closest (robust against a single noisy pixel).
         - exactly 1 valid zone: normally suppressed as untrustworthy, EXCEPT when
           it's near (<= LONE_NEAR_CM). A genuine thin/edge obstacle (wire, pole,
           table edge) may hit only one zone — suppressing a CLOSE lone hit would be
           a false-negative (the dangerous direction), so we trust it. A lone hit
           far away is still suppressed (likely noise, low avoidance stakes).
         Non-horizontal sensors stay at MAV_NO_DATA so they're skipped at send time. */
      sensor_ring_clear_one(idx);
      if (horiz[idx]) {
        for (uint8_t c = 0; c < COLS; c++) {
          if (FRONT_BLIND_DEG > 0.0f) {
            float bearing = yawDeg[idx] + colYaw4[c] + BEARING_SHIFT_DEG;
            float bmod    = fmodf(bearing + 360.0f, 360.0f);
            if (bmod < FRONT_BLIND_DEG || bmod > 360.0f - FRONT_BLIND_DEG) continue;
          }

          if (colCnt[c] >= 2 ||
              (colCnt[c] == 1 && colMin[c] <= LONE_NEAR_CM)) {
            sensorRing[idx][c] = colMin[c];          // trusted obstacle
          } else {
            sensorRing[idx][c] = uint16_t(MAX_CM + 1); // clear (no/insufficient evidence)
          }
        }
      }

      /* Stop after one successful read per loop iteration. Reading all
         ready sensors in one pass would cost up to ~40 ms (5 * ~8 ms of
         I2C) and stall the publish gate. Round-robin gives each sensor
         a turn over consecutive iterations — same per-bin refresh
         (bus-bound at ~25 Hz) but loop runs at >100 Hz so publishing
         at PUBLISH_HZ stays accurate. */
      break;
    }
    nextSens = (nextSens + 1) % NUM_SENS;

    /* Whole-bus failure detection: if every ACTIVE (non-excluded) sensor is
       down, the mux itself is likely gone (brownout/latch). Flag it so the
       recovery block above runs a full bus+mux re-init next iteration
       instead of the per-sensor reinit (which can't fix a dead mux).
       Excluded sensors don't count — a quarantined/disabled sensor being
       "down" is expected, not evidence of mux failure. Disabled entirely in
       solo-test mode: the one active sensor being dead may be the very
       thing under test and must not trigger bus-recovery churn. */
#if SOLO_TEST_SENSOR < 0
    bool anyActive = false, anyOk = false;
    for (uint8_t k = 0; k < NUM_SENS; k++) {
      if (sensorExcluded(k)) continue;
      anyActive = true;
      if (sensorOk[k]) { anyOk = true; break; }
    }
    if (anyActive && !anyOk) muxOk = false;
#endif
  }

  uint32_t now_us = micros();
  if ((uint32_t)(now_us - last_pub_us) >= PUBLISH_PERIOD_US) {
    /* Advance by exactly PUBLISH_PERIOD_US (not snap to now_us) so the
       publish rate averages the configured PUBLISH_HZ even though loop
       iterations don't divide evenly into the period. The stall guard
       below prevents a long catch-up burst after a hiccup. */
    last_pub_us += PUBLISH_PERIOD_US;
    /* If we're more than one period behind (true stall), skip ahead to
       avoid a long burst. */
    if ((uint32_t)(now_us - last_pub_us) > PUBLISH_PERIOD_US * 4) {
      last_pub_us = now_us;
    }
    pub_count_total++;
    sendHeartbeat();
    requestAttitudeStream();  // self-gated to every REQUEST_PERIOD_MS
    sendObstacleDistance();
    sendDistanceSensorTop();
    checkStatusTransitions();   // fires STATUSTEXT only on edges (mux/sensor/attitude)
    debug_print_line();
#if DEBUG_ZONES
    debug_zones_dump();
#endif
  }
}

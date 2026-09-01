#!/usr/bin/env python3
"""
flight_recorder -- auto rosbag recorder of the full SLAM + FC diagnostic topic set.

Records ONLY while the drone is ARMED:
  /mavros/state.armed  False -> True  : start a session
  /mavros/state.armed  True  -> False : stop  (clean SIGINT close)
A SLAM LAND/restart (/slam/arm_interlock=LOCKED) does NOT stop it -- we want to keep
recording through a drift/LAND event for post-flight diagnosis.

Session layout (on the thumbdrive):
  /mnt/usb/MM_DD_YYYY_HH_mm_ss_flight/<ts>_<N>.bag   (lz4, split every ~2 min)
captures Tier1+2: raw lidar+imu (offline SLAM replay), FAST-LIO odom/health, what we SEND
the FC (vision_pose/speed), and what the FC REPORTS back (local_position, altitude,
estimator_status, statustext, rc in/out, imu/vibe, setpoints, battery, tf, monitor verdicts).

SAFETY: READ-ONLY toward the FC. It only *reads* /mavros/state to know when armed. It NEVER
arms, never commands mode, never writes FC params. It cannot arm the vehicle.
"""
import os
import signal
import subprocess
import time
import rospy

try:
    from mavros_msgs.msg import State, StatusText
    _HAVE_MAVROS = True
except Exception:
    _HAVE_MAVROS = False

_SEV_INFO, _SEV_WARN = 6, 4

# Tier1 + Tier2 diagnostic topic set (override via ~topics)
_DEFAULT_TOPICS = " ".join([
    # --- Tier 1: SLAM in/out + send/receive + FC core ---
    "/lidar_points", "/lidar_imu",                       # raw -> offline SLAM replay
    "/Odometry", "/fastlio_health",                      # FAST-LIO estimate + drift metrics
    "/mavros/vision_pose/pose", "/mavros/vision_speed/speed_vector",  # what we SEND the FC
    "/mavros/local_position/pose", "/mavros/local_position/velocity_local",  # FC fused pos/vel
    "/mavros/altitude",                                  # EKF vs baro vs rel alt (climb evidence)
    "/mavros/estimator_status",                          # EKF health: is vision aiding?
    "/mavros/statustext/recv",                           # FC msgs: "stopped aiding", PreArm, FS
    "/mavros/state",                                     # mode / armed / ekf-connected
    "/mavros/rc/in", "/mavros/rc/out",                   # throttle stick + motor outputs (EMI/vibe corr.)
    # --- Tier 2: full picture ---
    "/mavros/imu/data", "/mavros/imu/data_raw",          # vibration cross-check
    "/mavros/vfr_hud",                                   # climb rate / throttle quick-look
    "/mavros/setpoint_raw/target_local",                 # what the controller is commanding
    "/mavros/local_position/odom", "/mavros/global_position/local",  # full FC nav state
    "/mavros/extended_state",                            # landed / in-air
    "/mavros/battery", "/mavros/sys_status",             # sag / load correlations
    "/mavros/timesync_status", "/mavros/time_reference", # link latency / health
    "/slam/drift_risk", "/slam/arm_interlock",           # monitor verdicts
    "/tf", "/tf_static",
])


class FlightRecorder:
    def __init__(self):
        rospy.init_node("flight_recorder", anonymous=False)
        g = rospy.get_param
        self.out_root = g("~out_root", "/mnt/usb")
        self.marker = g("~thumbdrive_marker", ".slam_thumbdrive")
        self.topics = g("~topics", _DEFAULT_TOPICS).split()
        self.split_min = float(g("~split_minutes", 2.0))
        self.trigger = g("~trigger", "arm")          # arm | always (test only)

        self.armed_prev = None       # None until first /mavros/state (no auto-start if booted armed)
        self.active = False
        self.session_dir = self.bag_base = None
        self.rec_proc = None

        self.st_pub = rospy.Publisher("/mavros/statustext/send", StatusText, queue_size=10) if _HAVE_MAVROS else None
        rospy.on_shutdown(lambda: self.stop_session("shutdown"))

        if self.trigger == "always":
            rospy.Timer(rospy.Duration(2.0), lambda _e: self.start_session(), oneshot=True)
        elif _HAVE_MAVROS:
            rospy.Subscriber("/mavros/state", State, self.state_cb, queue_size=10)

        rospy.loginfo("flight_recorder up. trigger=%s -> %s ; %d topics ; split %.0fmin ; record while ARMED",
                      self.trigger, self.out_root, len(self.topics), self.split_min)

    # ---- trigger ----
    def state_cb(self, msg):
        armed = bool(msg.armed)
        if self.armed_prev is None:          # first reading: init only, never auto-start
            self.armed_prev = armed
            return
        if armed and not self.armed_prev:    # DISARMED -> ARMED
            self.start_session()
        elif not armed and self.armed_prev:  # ARMED -> DISARMED
            self.stop_session("disarm")
        self.armed_prev = armed

    # ---- session ----
    def start_session(self):
        if self.active:
            return
        if not self._thumbdrive_ok():
            rospy.logwarn("flight_recorder: thumbdrive missing at %s -> NOT recording", self.out_root)
            self._st(_SEV_WARN, "Thumbdrive missing: NOT recording")
            return
        ts = time.strftime("%m_%d_%Y_%H_%M_%S")
        self.session_dir = os.path.join(self.out_root, ts + "_flight")
        try:
            os.makedirs(self.session_dir, exist_ok=True)
        except Exception as e:
            rospy.logerr("flight_recorder: mkdir failed: %s", e)
            self._st(_SEV_WARN, "Rec dir create FAILED")
            return
        self.bag_base = os.path.join(self.session_dir, ts)
        cmd = ["rosbag", "record", "--lz4",
               "--split", "--duration=%dm" % int(round(self.split_min)),
               "-O", self.bag_base] + self.topics
        self.rec_proc = subprocess.Popen(cmd)
        self.active = True
        rospy.logwarn("flight_recorder: SESSION START -> %s (%d topics)", self.session_dir, len(self.topics))
        self._st(_SEV_INFO, "recording started, %s" % self._freestr())

    def stop_session(self, reason):
        if not self.active:
            return
        self.active = False
        if self.rec_proc is not None and self.rec_proc.poll() is None:
            self.rec_proc.send_signal(signal.SIGINT)   # clean bag close (flush + index)
            try:
                self.rec_proc.wait(timeout=15)
            except Exception:
                self.rec_proc.kill()
        self.rec_proc = None
        rospy.logwarn("flight_recorder: SESSION STOP (%s)", reason)
        self._st(_SEV_INFO, "recording finished, %s" % self._freestr())

    # ---- helpers (same patterns as ch9_logger) ----
    def _thumbdrive_ok(self):
        return os.path.isfile(os.path.join(self.out_root, self.marker))

    def _freestr(self):
        try:
            import shutil
            path = self.session_dir if (self.session_dir and os.path.isdir(self.session_dir)) else self.out_root
            u = shutil.disk_usage(path)
            gb = 1024.0 ** 3
            pct = 100.0 * u.free / u.total if u.total else 0.0
            return "%.0f/%.0f GB (%.0f%% free)" % (u.free / gb, u.total / gb, pct)
        except Exception:
            return "free space n/a"

    def _st(self, sev, text):
        if self.st_pub is not None:
            self.st_pub.publish(StatusText(severity=sev, text=text[:49]))


if __name__ == "__main__":
    try:
        FlightRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

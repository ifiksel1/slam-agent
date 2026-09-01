#!/usr/bin/env python3
"""
ch9_logger -- RC-triggered rosbag + LAS map logger to the thumbdrive.

CH9 HIGH (>= ~1700)  -> start a session: make /mnt/usb/MM_DD_YYYY_HH_mm_ss/ and inside it:
    <ts>.bag : rosbag (lz4) of /lidar_points + /lidar_imu (continuous)
    <ts>.las : LAS snapshot of /Laser_map (the accumulated map), OVERWRITTEN every 30 s + a final write
CH9 LOW  (<= ~1300)  -> stop the session.
A LAND/restart (drift_monitor sets /slam/arm_interlock=LOCKED) also stops the session.

Notes / guards:
  - SEPARATE from drift_monitor. CH9 middle (LAND) is handled there with a hold-dwell so flicking
    LOW<->HIGH (transit through middle) does NOT trigger LAND.
  - Thumbdrive guard: only records if the real thumbdrive is mounted (verified via a marker file
    /mnt/usb/.slam_thumbdrive); otherwise posts a GCS warning and does NOT record (never the internal disk).
  - Edge-triggered: if CH9 is already HIGH at startup, waits for a fresh LOW->HIGH before starting.
  - READ-ONLY toward the FC: reads RC, records topics, writes files. No arming, no FC commands.
"""
import os
import signal
import subprocess
import time
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

try:
    from mavros_msgs.msg import RCIn, StatusText
    _HAVE_MAVROS = True
except Exception:
    _HAVE_MAVROS = False
try:
    import laspy
    _HAVE_LASPY = True
except Exception:
    _HAVE_LASPY = False

_SEV_INFO, _SEV_WARN = 6, 4


class Ch9Logger:
    def __init__(self):
        rospy.init_node("ch9_logger", anonymous=False)
        g = rospy.get_param
        self.ch9_channel = int(g("~ch9_channel", 9))
        self.high_thresh = int(g("~high_thresh", 1700))
        self.low_thresh = int(g("~low_thresh", 1300))
        self.out_root = g("~out_root", "/mnt/usb")
        self.marker = g("~thumbdrive_marker", ".slam_thumbdrive")
        self.las_topic = g("~las_topic", "/Laser_map")
        self.bag_topics = g("~bag_topics", "/lidar_points /lidar_imu").split()
        self.las_period = float(g("~las_period", 30.0))

        self.ch9_high = None          # None until first RC reading (no auto-start if booted HIGH)
        self.active = False
        self.session_dir = self.bag_path = self.las_path = None
        self.rec_proc = None
        self.las_timer = None
        self.latest_map = None
        self.interlock_prev = None

        self.st_pub = rospy.Publisher("/mavros/statustext/send", StatusText, queue_size=10) if _HAVE_MAVROS else None
        rospy.Subscriber(self.las_topic, PointCloud2, lambda m: setattr(self, "latest_map", m), queue_size=1)
        if _HAVE_MAVROS:
            rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_cb, queue_size=10)
        rospy.Subscriber("/slam/arm_interlock", String, self.interlock_cb, queue_size=10)
        rospy.on_shutdown(lambda: self.stop_session("shutdown"))
        rospy.loginfo("ch9_logger up. CH%d HIGH>=%d start / LOW<=%d stop -> %s ; LAS %s every %.0fs ; laspy=%s",
                      self.ch9_channel, self.high_thresh, self.low_thresh, self.out_root,
                      self.las_topic, self.las_period, _HAVE_LASPY)

    # ---- triggers ----
    def rc_cb(self, msg):
        idx = self.ch9_channel - 1
        if idx < 0 or idx >= len(msg.channels):
            return
        v = msg.channels[idx]
        if v == 0:
            return
        is_high = v >= self.high_thresh
        is_low = v <= self.low_thresh
        if self.ch9_high is None:          # first reading: init only, never auto-start
            self.ch9_high = is_high
            return
        if is_high and not self.ch9_high:  # rising edge into HIGH
            self.ch9_high = True
            self.start_session()
        elif is_low:
            self.ch9_high = False
            self.stop_session("CH%d LOW" % self.ch9_channel)
        elif not is_high:
            self.ch9_high = False          # re-arm in the middle (so a later HIGH is a fresh edge)

    def interlock_cb(self, msg):
        if msg.data == "LOCKED" and self.interlock_prev != "LOCKED" and self.active:
            self.stop_session("LAND/restart")
        self.interlock_prev = msg.data

    # ---- session ----
    def start_session(self):
        if self.active:
            return
        if not self._thumbdrive_ok():
            rospy.logwarn("ch9_logger: thumbdrive not present at %s -> NOT recording", self.out_root)
            self._st(_SEV_WARN, "Thumbdrive missing: NOT recording")
            return
        ts = time.strftime("%m_%d_%Y_%H_%M_%S")
        self.session_dir = os.path.join(self.out_root, ts)
        try:
            os.makedirs(self.session_dir, exist_ok=True)
        except Exception as e:
            rospy.logerr("ch9_logger: mkdir failed: %s", e); self._st(_SEV_WARN, "Log dir create FAILED"); return
        self.bag_path = os.path.join(self.session_dir, ts + ".bag")
        self.las_path = os.path.join(self.session_dir, ts + ".las")
        # continuous rosbag (lz4); Popen so we can SIGINT it for a clean close
        cmd = ["rosbag", "record", "--lz4", "-O", self.bag_path] + self.bag_topics
        self.rec_proc = subprocess.Popen(cmd)
        self.active = True
        self._write_las()  # immediate first snapshot
        self.las_timer = rospy.Timer(rospy.Duration(self.las_period), lambda _e: self._write_las())
        rospy.logwarn("ch9_logger: SESSION START -> %s", self.session_dir)
        self._st(_SEV_INFO, "recording started, %s" % self._freestr())

    def stop_session(self, reason):
        if not self.active:
            return
        self.active = False
        if self.las_timer is not None:
            self.las_timer.shutdown(); self.las_timer = None
        if self.rec_proc is not None and self.rec_proc.poll() is None:
            self.rec_proc.send_signal(signal.SIGINT)   # clean bag close
            try:
                self.rec_proc.wait(timeout=10)
            except Exception:
                self.rec_proc.kill()
        self.rec_proc = None
        self._write_las()  # final snapshot
        rospy.logwarn("ch9_logger: SESSION STOP (%s)", reason)
        self._st(_SEV_INFO, "recording finished, %s" % self._freestr())

    # ---- helpers ----
    def _thumbdrive_ok(self):
        # real thumbdrive is mounted iff the marker file is visible (else /mnt/usb is the internal fallback dir)
        return os.path.isfile(os.path.join(self.out_root, self.marker))

    def _freestr(self):
        # "free space left: X/Y GB (Z% free)" for the GCS banner (GiB, matches df -h)
        try:
            import shutil
            path = self.session_dir if (self.session_dir and os.path.isdir(self.session_dir)) else self.out_root
            u = shutil.disk_usage(path)
            gb = 1024.0 ** 3
            pct = 100.0 * u.free / u.total if u.total else 0.0
            return "%.0f/%.0f GB (%.0f%% free)" % (u.free / gb, u.total / gb, pct)
        except Exception:
            return "free space n/a"

    def _write_las(self):
        if not _HAVE_LASPY or self.las_path is None:
            return
        m = self.latest_map
        if m is None:
            return
        try:
            fields = [f.name for f in m.fields]
            use_i = "intensity" in fields
            names = ("x", "y", "z", "intensity") if use_i else ("x", "y", "z")
            pts = np.array(list(pc2.read_points(m, field_names=names, skip_nans=True)), dtype=np.float64)
            if pts.size == 0:
                return
            hdr = laspy.LasHeader(point_format=3)
            hdr.offsets = np.min(pts[:, :3], axis=0)
            hdr.scales = [0.001, 0.001, 0.001]
            las = laspy.LasData(hdr)
            las.x = pts[:, 0]; las.y = pts[:, 1]; las.z = pts[:, 2]
            if use_i:
                inten = pts[:, 3]
                rng = inten.max() - inten.min()
                las.intensity = ((inten - inten.min()) / rng * 65535).astype(np.uint16) if rng > 0 \
                    else np.zeros(len(inten), dtype=np.uint16)
            tmp = self.las_path + ".tmp"
            las.write(tmp); os.replace(tmp, self.las_path)   # atomic overwrite
        except Exception as e:
            rospy.logerr("ch9_logger: LAS write failed: %s", e)

    def _st(self, sev, text):
        if self.st_pub is not None:
            self.st_pub.publish(StatusText(severity=sev, text=text[:49]))


if __name__ == "__main__":
    try:
        Ch9Logger(); rospy.spin()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3
# Live SLAM metrics from Adaptive-LIO /odom (nav_msgs/Odometry).
# Prints position, orientation, velocity, speed, distance travelled, displacement-from-start and
# peak excursion, refreshed ~2 Hz, and logs every sample to CSV. Ctrl-C prints a summary.
#
# By default it applies the pitch-180 mount correction (Ouster is upside-down) so x/y/z read upright
# (z = up); pass --raw to see the raw alio_odom frame. Distance/speed are frame-invariant either way.
#
# Usage:
#   ./slam_metrics.sh                          # convenience wrapper (sets ROS env + master)
#   python3 slam_metrics.py [--topic /odom] [--csv out.csv] [--raw]
import sys, math, time, signal

# ---- args -------------------------------------------------------------
def arg(flag, default=None):
    return sys.argv[sys.argv.index(flag) + 1] if flag in sys.argv else default
TOPIC = arg('--topic', '/odom')
CSV   = arg('--csv', '/tmp/slam_metrics.csv')
RAW   = '--raw' in sys.argv

import rospy
from nav_msgs.msg import Odometry

def quat_to_rpy(x, y, z, w):
    # roll (x), pitch (y), yaw (z), radians
    r = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    sp = 2*(w*y - z*x); sp = max(-1.0, min(1.0, sp)); p = math.asin(sp)
    yw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return r, p, yw

class Metrics:
    def __init__(self):
        self.t0 = None; self.p0 = None; self.prev_p = None; self.prev_t = None
        self.dist = 0.0; self.peak = 0.0; self.vmax = 0.0; self.n = 0
        self.last_print = 0.0
        # Adaptive-LIO's own extras (optional)
        self.alio_vel = None; self.alio_dist = None
        self.f = open(CSV, 'w')
        self.f.write("t,x,y,z,roll_deg,pitch_deg,yaw_deg,vx,vy,vz,speed,dist_travelled,disp_from_start\n")
        rospy.Subscriber(TOPIC, Odometry, self.cb, queue_size=200)
        try:
            from std_msgs.msg import Float32
            rospy.Subscriber('/velocity',  Float32, lambda m: setattr(self, 'alio_vel',  m.data), queue_size=5)
            rospy.Subscriber('/move_dist', Float32, lambda m: setattr(self, 'alio_dist', m.data), queue_size=5)
        except Exception:
            pass

    def cb(self, m):
        t = m.header.stamp.to_sec()
        if self.t0 is None: self.t0 = t
        rt = t - self.t0
        p = m.pose.pose.position; q = m.pose.pose.orientation
        x, y, z = p.x, p.y, p.z
        if not RAW:                       # pitch-180 (upside-down mount): (x,y,z)->(-x,y,-z)
            x, z = -x, -z
        roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        if not RAW:                       # rotate orientation into the upright frame too
            pitch = -pitch; yaw = -yaw

        if self.p0 is None: self.p0 = (x, y, z)
        disp = math.dist((x, y, z), self.p0)
        self.peak = max(self.peak, disp)

        # velocity: prefer /odom twist if the estimator fills it, else finite-difference position
        tw = m.twist.twist.linear
        vx, vy, vz = tw.x, tw.y, tw.z
        if not RAW: vx, vz = -vx, -vz
        speed = math.sqrt(vx*vx + vy*vy + vz*vz)
        step = 0.0
        if self.prev_p is not None and self.prev_t is not None:
            dt = t - self.prev_t
            step = math.dist((x, y, z), self.prev_p)
            self.dist += step
            if speed == 0.0 and 0 < dt < 1.0:   # twist empty -> derive from position delta
                vx = (x - self.prev_p[0]) / dt; vy = (y - self.prev_p[1]) / dt; vz = (z - self.prev_p[2]) / dt
                speed = step / dt if dt > 0 else 0.0
        self.vmax = max(self.vmax, speed)
        self.prev_p = (x, y, z); self.prev_t = t; self.n += 1

        self.f.write(f"{rt:.3f},{x:.4f},{y:.4f},{z:.4f},{math.degrees(roll):.2f},{math.degrees(pitch):.2f},"
                     f"{math.degrees(yaw):.2f},{vx:.3f},{vy:.3f},{vz:.3f},{speed:.3f},{self.dist:.4f},{disp:.4f}\n")

        now = time.time()
        if now - self.last_print >= 0.5:
            self.last_print = now
            warn = "  !!JUMP" if step > 0.5 else ""
            extra = ""
            if self.alio_vel is not None:  extra += f"  alio_v={self.alio_vel:.2f}"
            if self.alio_dist is not None: extra += f"  alio_dist={self.alio_dist:.1f}"
            sys.stdout.write(
                f"\rt={rt:6.1f}  pos=({x:+6.2f},{y:+6.2f},{z:+6.2f})m  "
                f"rpy=({math.degrees(roll):+5.0f},{math.degrees(pitch):+5.0f},{math.degrees(yaw):+5.0f})deg  "
                f"v={speed:4.2f}(max{self.vmax:4.2f})m/s  dist={self.dist:6.2f}m  "
                f"disp={disp:5.2f}(peak{self.peak:5.2f})m{extra}{warn}   ")
            sys.stdout.flush()

def main():
    rospy.init_node('slam_metrics', anonymous=True, disable_signals=True)
    m = Metrics()
    frame = "raw alio_odom" if RAW else "upright (pitch-180 corrected)"
    print(f"SLAM metrics on {TOPIC}  [frame: {frame}]  csv={CSV}\n(Ctrl-C for summary)\n")
    stop = {'v': False}
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, lambda *a: stop.update(v=True))
    while not rospy.is_shutdown() and not stop['v']:
        time.sleep(0.1)
    m.f.flush(); m.f.close()
    dispf = math.dist(m.prev_p, m.p0) if (m.prev_p and m.p0) else 0.0
    print("\n\n==== SLAM SUMMARY ====")
    print(f" samples            : {m.n}")
    print(f" distance travelled : {m.dist:.2f} m")
    print(f" final displacement : {dispf:.2f} m   (from start)")
    print(f" peak excursion     : {m.peak:.2f} m")
    print(f" max speed          : {m.vmax:.2f} m/s")
    if m.prev_p: print(f" final position     : ({m.prev_p[0]:+.2f}, {m.prev_p[1]:+.2f}, {m.prev_p[2]:+.2f}) m")
    print(f" csv                : {CSV}")

if __name__ == '__main__':
    main()

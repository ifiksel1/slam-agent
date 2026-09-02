#!/usr/bin/env python3
"""
fc_statustext.py - print STATUSTEXT from the flight controller.

Use this instead of `rostopic echo /mavros/statustext/recv`. On this stack that topic is
advertised by /mavros but never publishes: on 2026-09-01 the FC sent STATUSTEXT (msgid 253 is
plainly visible on /mavlink/from) while the plugin's topic stayed empty through a scripting
restart, two mode changes and a prearm-check run. Reading the raw stream sidesteps the plugin.

This is not a nicety. It is the only reason the Lua arming gate's failure was diagnosable: the FC
had been reporting

    Lua: /APM/scripts/slam_latency_gate.lua:41: SLG: could not add param table

at every boot, and nothing on the companion could see it. A script that dies at load looks
exactly like a script that was never installed.

Usage:
    python3 scripts/fc_statustext.py [seconds]        # default 30, 0 = forever

Run it inside the container:
    docker exec slam-hesai-fastlio bash -lc \
      'source /opt/ros/noetic/setup.bash; source /root/slam_ws/devel/setup.bash; \
       python3 /root/slam_ws/src/FAST_LIO_SLAM/FAST_LIO_SLAM/FAST-LIO/scripts/fc_statustext.py 30'

Tip: ArduPilot chunks statustext at 50 characters, so a long message arrives as consecutive
lines and reads as though it were truncated. It is not - join the pieces.
"""
import struct
import sys

import rospy
from mavros_msgs.msg import Mavlink

STATUSTEXT_MSGID = 253
SEVERITY = {0: "EMERG", 1: "ALERT", 2: "CRIT", 3: "ERR",
            4: "WARN", 5: "NOTICE", 6: "INFO", 7: "DEBUG"}


def on_mavlink(msg):
    if msg.msgid != STATUSTEXT_MSGID:
        return
    raw = b"".join(struct.pack("<Q", w) for w in msg.payload64)
    severity = raw[0]
    text = raw[1:51].split(b"\x00")[0].decode("ascii", "replace")
    stamp = rospy.get_time()
    print("[%10.2f] [%-6s] %s" % (stamp, SEVERITY.get(severity, severity), text))
    sys.stdout.flush()


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    rospy.init_node("fc_statustext", anonymous=True)
    rospy.Subscriber("/mavlink/from", Mavlink, on_mavlink)
    if duration > 0:
        rospy.sleep(duration)
    else:
        rospy.spin()


if __name__ == "__main__":
    main()

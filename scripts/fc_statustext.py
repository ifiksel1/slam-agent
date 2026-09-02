#!/usr/bin/env python3
"""
fc_statustext.py - print STATUSTEXT from the flight controller.

Reads STATUSTEXT off the raw /mavlink/from stream rather than /mavros/statustext/recv.

CORRECTION (2026-09-02): an earlier version of this docstring claimed the mavros statustext
plugin "never publishes". That was wrong. A same-window comparison of both subscriptions saw
7 raw msgid-253 messages and the same 7 on /mavros/statustext/recv, identical. The windows that
looked empty simply contained no FC messages - ArduPilot is silent when it has nothing to say,
and a MAVLink-commanded mode change emits nothing at all. `rostopic echo
/mavros/statustext/recv` is perfectly good for normal use.

What this script is still for: decoding the raw stream when you need to be certain you are
seeing everything the FC sent, without a plugin between you and the wire - and for filtering by
content, which the plain topic echo cannot do. It is also the form the diagnosis below was made
in. The FC had been reporting

    Lua: /APM/scripts/slam_latency_gate.lua:41: SLG: could not add param table

at every boot, and it went unseen for a different reason worth knowing: an FC reboot
re-enumerates the USB device, mavros does not reconnect on its own, and by the time the container
is restarted the boot banners are long gone. So FC boot messages are missed by default no matter
which topic you watch. A script that dies at load then looks exactly like one that was never
installed - no params, no messages, arming silently ungated.

To catch boot-time output, start this (or the topic echo) BEFORE power-cycling the FC, or use
MAV_CMD_SCRIPTING to reload scripts on a live link - but see the 4.6.3 aux-auth landmine in
fc_scripts/slam_latency_gate.lua before doing that with any script that claims a prearm slot.

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

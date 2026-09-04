#!/usr/bin/env python3
"""Pull a dataflash log off the FC over MAVLink LOG_DATA.

Usage: pull_fc_log.py <log_id> <out.BIN> [device]

Requests the whole log in one go, then re-requests any byte ranges that never
arrived. The FC streams LOG_DATA as fast as the link allows; over USB CDC the
nominal 115200 baud is not the real ceiling, so this is usually link-limited
rather than baud-limited.
"""
import sys
import time

from pymavlink import mavutil

log_id = int(sys.argv[1])
out = sys.argv[2]
dev = sys.argv[3] if len(sys.argv) > 3 else '/dev/ttyACM0'

m = mavutil.mavlink_connection(dev, baud=115200)
m.wait_heartbeat(timeout=20)
sys.stderr.write("heartbeat sys=%d comp=%d\n" % (m.target_system, m.target_component))

# ask for just this log's entry to learn its size
m.mav.log_request_list_send(m.target_system, m.target_component, log_id, log_id)
size = None
t0 = time.time()
while time.time() - t0 < 15:
    e = m.recv_match(type='LOG_ENTRY', blocking=True, timeout=3)
    if e is not None and e.id == log_id:
        size = e.size
        break
if not size:
    sys.exit("no LOG_ENTRY for id %d" % log_id)
sys.stderr.write("log %d size %d bytes\n" % (log_id, size))

buf = bytearray(size)
have = bytearray(size)          # byte-level coverage map
CHUNK = 90


def request(ofs, count):
    m.mav.log_request_data_send(m.target_system, m.target_component, log_id, ofs, count)


def missing_ranges():
    ranges, start = [], None
    for i in range(size):
        if not have[i] and start is None:
            start = i
        elif have[i] and start is not None:
            ranges.append((start, i - start))
            start = None
    if start is not None:
        ranges.append((start, size - start))
    return ranges


request(0, size)
last_rx = time.time()
got = 0
t_start = time.time()
passes = 0

while True:
    msg = m.recv_match(type='LOG_DATA', blocking=True, timeout=1.0)
    if msg is not None and msg.id == log_id:
        n, o = msg.count, msg.ofs
        if o + n <= size:
            buf[o:o + n] = bytes(msg.data[:n])
            for i in range(o, o + n):
                if not have[i]:
                    have[i] = 1
                    got += 1
        last_rx = time.time()
        if got % (CHUNK * 500) < CHUNK:
            el = time.time() - t_start
            sys.stderr.write("\r%.1f%%  %d/%d  %.0f kB/s  %.0fs" %
                             (100.0 * got / size, got, size,
                              got / 1024.0 / max(el, 0.1), el))
            sys.stderr.flush()
    if got >= size:
        break
    if time.time() - last_rx > 3.0:
        gaps = missing_ranges()
        if not gaps:
            break
        passes += 1
        if passes > 400:
            sys.stderr.write("\ngiving up with %d ranges missing\n" % len(gaps))
            break
        sys.stderr.write("\n[gap pass %d] %d ranges, %d bytes missing; re-requesting\n"
                         % (passes, len(gaps), size - got))
        for (o, ln) in gaps[:20]:
            request(o, ln)
        last_rx = time.time()

with open(out, 'wb') as f:
    f.write(bytes(buf))
sys.stderr.write("\nwrote %s (%d bytes, %d covered)\n" % (out, size, got))
m.mav.log_request_end_send(m.target_system, m.target_component)

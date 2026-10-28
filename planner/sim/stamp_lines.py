"""Prefix each stdin line with the wall time (s), for logs whose lines carry no timestamps (SUPER's)."""
import sys
import time

for line in sys.stdin:
    sys.stdout.write("%.3f %s" % (time.time(), line))
    sys.stdout.flush()

#!/bin/bash
# Set Jetson Orin NX CPU clock cap. Usage: sudo ./set_cpu_clock.sh stock|boost
# stock = 1497600 kHz (nvpmodel 25W default)   boost = 1984000 kHz (silicon max, sysfs override)
# Non-persistent (reverts on reboot).
case "${1:-}" in
  stock) F=1497600 ;;
  boost) F=1984000 ;;
  *) echo "usage: sudo $0 stock|boost"; exit 1 ;;
esac
for c in 0 1 2 3 4 5; do echo $F > /sys/devices/system/cpu/cpu$c/cpufreq/scaling_max_freq; done
echo "scaling_max_freq set to $F on all 6 cores"
echo "cur: $(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq | tr '\n' ' ')"

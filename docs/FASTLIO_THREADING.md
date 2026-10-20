# FAST-LIO threading config (4 threads, pinned, passive waits)

Deployed on robot 4.1.4 (nuc-04) on 2026-10-18. This document explains what the config is,
why it was chosen, how to put it on another robot, how to check it, and how to roll it back.

## TL;DR

| Setting | Before | Now | Where it lives |
|---|---|---|---|
| OpenMP threads in the point-matching loop | 3 (`MP_PROC_NUM=3`) | 4 (`MP_PROC_NUM=4`) | `overrides/FAST-LIO/CMakeLists.txt` (compile time) |
| CPUs `fastlio_mapping` may run on | any of 0-11 | 0, 2, 4, 6 (one thread per fast physical core) | `config/fastlio_to_fc.launch`, `launch-prefix="taskset -c 0,2,4,6"` |
| What idle OpenMP workers do | spin (busy-wait) | sleep | `config/fastlio_to_fc.launch`, `<env name="OMP_WAIT_POLICY" value="passive"/>` |

Measured on a 1x replay of `jetson_cart_hallway` (50 s, confined profile), against the
original stock threading:

| | Stock (Oct 13) | This config | Change |
|---|---|---|---|
| Update time per scan, mean | 14.1 ms | 11.4 ms | -19% |
| Update time per scan, p95 | 22.4 ms | 17.2 ms | -23% |
| Worst scan | 32.8 ms | 20.1 ms | -39% |
| CPU time burned by FAST-LIO per run | 53.9 s | 32.4 s | -40% |
| Main thread time on slow cores | 11% | 0% | |
| Scan-match strength (eig_min median) | 770k | 770k | unchanged |

Trajectories stayed within normal run-to-run spread (stock runs on this bag alone differ by
up to 0.81 m at the end point).

## Background

- **CPU (nuc-04):** Intel Core 5 210H. CPUs 0-7 are 4 fast performance cores with 2 hyperthreads
  each (0/1, 2/3, 4/5, 6/7, 4.8 GHz). CPUs 8-11 are 4 slow efficiency cores (3.6 GHz).
  Check any other robot with `lscpu -e=CPU,CORE,MAXMHZ` before copying the CPU list.
- **Where FAST-LIO uses threads:** only the point-matching loop in `laserMapping.cpp`
  (`omp_set_num_threads(MP_PROC_NUM)` + `#pragma omp parallel for`: nearest map points, plane fit,
  residual per scan point). Downsampling, the IEKF update and the ikd-tree insert run on the main
  thread. So more threads speed up one part of each scan, and the gain flattens out.
- **`MP_PROC_NUM` is set at compile time** by `CMakeLists.txt` from the CPU count CMake sees
  (`ProcessorCount`). The `N GREATER 4` branch now selects 4.
- **Compute vs CPU:** "update time" is wall-clock time per scan (how fast). "CPU per run" is total
  CPU seconds all FAST-LIO threads used over the bag (how expensive). Spinning workers burn CPU
  without making scans faster, which is why passive waits cut CPU by about 40% at no speed cost.

## How it was chosen

All runs: 1x replay, `jetson_cart_hallway`, Claude confined profile (scan voxel 0.115, map voxel
0.11, every 3rd point, 3 iterations, blind 0.155, det_range 62.6, cube 212), same 50 ms thread
sampler, live SLAM stopped during benchmarks.

| Variant | Mean | p95 | Max | CPU / run |
|---|---|---|---|---|
| Stock 3 threads, unpinned (Oct 13) | 14.1 | 22.4 | 32.8 | 53.9 s |
| 3 threads, pinned 0,2,4,6, passive (Oct 17) | 13.7 | 20.4 | 26.9 | 32.5 s |
| 3 threads, pinned, passive, re-run (control, Oct 18) | 13.8 | 20.2 | 24.9 | 32.6 s |
| **4 threads, pinned 0,2,4,6, passive** | **11.4** | **17.2** | **20.1** | **32.4 s** |
| 4 threads, CPUs 0-7, passive | 12.1 | 18.9 | 22.6 | 33.1 s |
| 6 threads, CPUs 0-7, passive | 11.2 | 16.7 | 20.8 | 38.0 s |

(ms; one run per variant on Oct 18.) 6 threads is marginally faster but costs 17% more CPU, so
4 threads on 0,2,4,6 is the best value. Pinning each OpenMP thread to a fixed core
(`OMP_PROC_BIND`/`OMP_PLACES`) was worse on Oct 17 and is not used.

Caveats: single runs per variant on one confined bag. The heaviest case (the 20 Hz TQ4 flight
on the `open` profile) gained only ~4% from threading on Oct 17; that bag needs a lighter profile,
not more threads.

## What this PR changes

1. `overrides/FAST-LIO/CMakeLists.txt`: `MP_PROC_NUM` 3 -> 4 in the `N GREATER 4` branch.
   After a fresh `scripts/fetch_docker_sources.sh` (which applies overrides) and an image build,
   new images get 4 threads.
2. `config/fastlio_to_fc.launch`: the `laserMapping` node gets
   `launch-prefix="taskset -c 0,2,4,6"` and `<env name="OMP_WAIT_POLICY" value="passive"/>`.
   This file is bind-mounted into the container, so it applies on the next SLAM start, and the
   pinning survives the drift monitor's respawn of the node.
3. `scripts/threading/apply_thread_config.sh`: check / build / restart helper for robots that
   already have an image (no full image rebuild needed).
4. This document.

Nothing else in the launch file changes (profile params, drift response, recorders untouched).

## Porting to another robot

On the robot, in the slam-agent checkout, after pulling this branch:

```bash
scripts/threading/apply_thread_config.sh check     # read-only: CPU layout, compiled threads, what is running
scripts/threading/apply_thread_config.sh build     # MP_PROC_NUM=4 in the container + catkin build fast_lio
scripts/threading/apply_thread_config.sh restart   # only when disarmed; restarts slam-fastlio and verifies
```

1. **Check the CPU layout first.** `check` prints `lscpu -e`. If the robot is not a Core 5 210H
   with the same layout, choose one CPU per fast physical core and run with `CPUS=...`
   (and edit the `taskset` list in the launch file to match).
2. **Build without any CPU restriction.** Never run the build under `taskset` or a container
   cpuset: CMake would see fewer CPUs and silently compile 2 threads. `build` verifies
   `flags.make` shows `-DMP_PROC_NUM=4` and keeps the old binary as
   `fastlio_mapping.bak-pre-threads-<date>`.
3. **Restart only when disarmed.** `restart` refuses unless the GUI reports `armed: False`.
4. **Make it survive a container recreate (optional but recommended):** the rebuilt binary lives
   in the container layer. To keep it if the container is recreated from the image:
   ```bash
   docker tag slam-system:latest slam-system:pre-threads-$(date +%F)
   docker commit slam-hesai-fastlio slam-system:latest
   ```
   Check disk first (`df -h /`): the commit includes the container layer, including
   `/root/.ros` logs (about 4 GB on nuc-04).

Manual equivalent of `build`:

```bash
docker exec -it slam-hesai-fastlio bash
C=/root/slam_ws/src/FAST_LIO_SLAM/FAST_LIO_SLAM/FAST-LIO/CMakeLists.txt
sed -i '0,/add_definitions(-DMP_PROC_NUM=3)/s//add_definitions(-DMP_PROC_NUM=4)/' $C
source /opt/ros/noetic/setup.bash && cd /root/slam_ws && catkin build fast_lio
grep -o 'MP_PROC_NUM=[0-9]*' build/fast_lio/CMakeFiles/fastlio_mapping.dir/flags.make   # expect 4
```

## Verifying on a running robot

```bash
docker exec slam-hesai-fastlio bash -c '
  p=$(pgrep -x fastlio_mapping)
  grep Cpus_allowed_list /proc/$p/status                     # 0,2,4,6
  tr "\0" "\n" </proc/$p/environ | grep OMP_WAIT_POLICY       # passive
  ps -L -o tid,psr,pcpu,comm -p $p | sort -k3 -nr | head -5'  # main + 3 workers with similar %CPU
```

Also confirm `/Odometry` is about 20 Hz and the GUI shows the expected mode with ~15 nodes.

## Gotcha: a leftover ROS master on port 11311

All containers use the host network, so a `roscore` left running by bag replays (for example in
`fbt-tuner`) owns port 11311. A `systemctl restart slam-fastlio` then adopts that master instead
of starting its own, and when it exits every live node is orphaned (GUI shows 0 nodes,
`rostopic` reports "Unable to communicate with master") while the processes keep running.
Before restarting live SLAM after replay work, `ss -ltnp | grep 11311` must be empty. The
`restart` subcommand checks this.

## Rollback

```bash
# binary only (keeps the launch-file pinning):
docker exec slam-hesai-fastlio bash -c 'cd /root/slam_ws/devel/.private/fast_lio/lib/fast_lio && cp -p fastlio_mapping.bak-pre-threads-<date> fastlio_mapping'
# or the whole image (if you committed one):
docker tag slam-system:pre-threads-<date> slam-system:latest
# launch file: remove launch-prefix and the OMP env line from the laserMapping node
sudo systemctl restart slam-fastlio      # disarmed only
```

## Not done yet

- Two more runs per variant to put error bars on the numbers.
- Moving the lidar driver, MAVROS, bridge and recorders to CPUs 8-11 (untested on the live stack).
- Parallelizing more of FAST-LIO (two extra independent per-point loops are patched in a test
  workspace but not benchmarked).

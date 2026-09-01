# overrides/

The tracked copy of every file we have changed or added in the vendored ROS sources.

## Why this exists

`docker_src/` is `.gitignored` and re-created by `scripts/fetch_docker_sources.sh`, which
clones upstream. Until now our modifications lived **only** in that ignored working tree and
in the baked `slam-system:latest` image layer. A fresh clone of this repo, followed by the
documented fetch-and-build, produced a system with none of them — about 1,000 lines of work,
including `drift_monitor.py` and the `/fastlio_health` instrumentation in `laserMapping.cpp`,
one `docker system prune` away from being gone.

Tracking the whole of `docker_src/` was the alternative. It is 369 MB across three nested git
repos, and two of those three (`livox_ros_driver`, `vision_to_mavros`) are **completely
unmodified upstream**. Only `FAST_LIO_SLAM` differs, and only in eight files. So we track the
eight files, not the 369 MB.

## Layout

`overrides/FAST-LIO/` mirrors `docker_src/FAST_LIO_SLAM/FAST_LIO_SLAM/FAST-LIO/` exactly.
`UPSTREAM.yaml` pins the three vendored repos to the commits this system was built and flown
against, and lists what each override contains.

## Working with it

```bash
scripts/apply_overrides.sh check     # do the two copies agree? (exits 1 on drift)
scripts/apply_overrides.sh capture   # docker_src -> overrides, after editing live
scripts/apply_overrides.sh apply     # overrides -> docker_src, after a fresh fetch
```

The live edit loop is unchanged: edit in `docker_src/`, test, then run `capture` and commit.
`drift_monitor.py` is additionally bind-mounted into the container by `docker-compose.yml`, so
editing it there takes effect on a node restart with no rebuild — `capture` afterwards is what
makes the change durable.

Run `check` before any commit that touches the SLAM stack. Drift between the two copies means
the repo is describing a system that is not the one running.

## Rebuilding from nothing

```bash
gh auth login && gh auth setup-git      # FAST_LIO_SLAM is private
scripts/fetch_docker_sources.sh          # clones + checks out the pinned SHAs, then applies overrides
docker compose build
```

## What this deliberately does not do

It is a **whole-file override layer, not a patch series.** Moving to a newer upstream means
re-doing the merge by hand, and `laserMapping.cpp` is a 145-line delta against a large file —
expect that to be real work. The trade is intentional: the thing worth protecting is our
modifications, not the ability to rebase them cheaply, and a patch series that fails to apply
protects nothing.

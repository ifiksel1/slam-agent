# COIN-LIO ⊕ EllipseLIO photometric-fusion DEV image — ROS 2 Humble, ARM64/Jetson.
#
# This is a DEVELOPMENT container for the port that fuses COIN-LIO's LiDAR-intensity
# direct-photometric residual into EllipseLIO's IKFoM measurement model (see
# slam-agent/ellipselio_integration/docs/coinlio_photometric_port_plan.md).
#
# It inherits everything from the proven ellipselio:humble runtime (C++17 toolchain,
# OpenMP, Eigen, PCL, OpenCV, cv_bridge, CycloneDDS, Ouster ROS2 driver deps) and only
# adds editor/navigation tooling for in-container source work. Kept separate from the
# production `ellipselio` container so the port never touches the validated runtime.
FROM ellipselio:humble

# Dev-navigation conveniences for working the FAST-LIO2/IKFoM port in-container.
# Tolerant of transient apt mirror failures — the image is still usable without these.
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        ripgrep gdb clangd clang-format bear python3-pip \
 || echo "WARN: optional dev tools apt step failed (network?) — continuing" \
 ; apt-get clean && rm -rf /var/lib/apt/lists/*

# Marker so `docker ps`/inspect makes the purpose obvious.
LABEL org.ultraview.project="coinlio-photometric-fusion" \
      org.ultraview.base="ellipselio:humble" \
      org.ultraview.note="dev container for COIN-LIO photometric residual -> EllipseLIO IKFoM port"

ENV ELLIPSE_PHOTOMETRIC_DEV=1
WORKDIR /root/ros2_ws

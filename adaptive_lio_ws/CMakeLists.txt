cmake_minimum_required(VERSION 3.0.2)
project(adaptive_lio_ws)

# Config/launch-only integration package — no compiled targets. Just installs the helper
# scripts so `rosrun adaptive_lio_ws <script>.py` works and marks the package for $(find).
find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/adaptive_lio_traj_sampler.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

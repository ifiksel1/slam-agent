# Troubleshooting: ROS Environment Issues

## "Package not found"
```bash
# Check sourcing
echo $ROS_PACKAGE_PATH  # Should include your workspace
source ~/catkin_ws/devel/setup.bash  # Re-source

# Check if built
catkin build PACKAGE_NAME

# Check if in workspace
ls ~/catkin_ws/src/PACKAGE_NAME/
```

## Build Errors
| Error | Fix |
|-------|-----|
| "Could not find package X" | `sudo apt install ros-$ROS_DISTRO-X` (replace _ with -) |
| "CMake Error: Could not find Eigen3" | `sudo apt install libeigen3-dev` |
| "PCL not found" | `sudo apt install libpcl-dev ros-$ROS_DISTRO-pcl-ros` |
| "GTSAM not found" | `sudo add-apt-repository ppa:borglab/gtsam-release-4.0 && sudo apt install libgtsam-dev` |
| "cv_bridge not found" | `sudo apt install ros-$ROS_DISTRO-cv-bridge` |
| "tf2 not found" | `sudo apt install ros-$ROS_DISTRO-tf2-ros` |
| "Ceres not found" | Build from source: `git clone https://ceres-solver.googlesource.com/ceres-solver && mkdir build && cd build && cmake .. && make -j4 && sudo make install` |
| Multiple workspace conflicts | Source workspaces in correct order (system then base then user) |

## ROS_MASTER_URI
```bash
# Single machine
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1

# Multi-machine
export ROS_MASTER_URI=http://MASTER_IP:11311
export ROS_IP=THIS_MACHINE_IP
```

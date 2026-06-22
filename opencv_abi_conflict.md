# OpenCV 4.2/4.5 ABI Conflict on Jetson + ROS Noetic

## Problem
COIN-LIO crashes with `cv::Exception: s >= 0 in function 'setSize'` on Jetson Orin NX.

## Root Cause
ROS Noetic's `cv_bridge` transitively loads `libopencv_imgproc.so.4.2` while the system has OpenCV 4.5.
When COIN-LIO's `ImageProcessor::removeLines()` calls `cv::filter2D`, it uses the 4.2 version of `filter2D`
but passes a `cv::Mat` created by 4.5. The internal `cv::Mat` struct layout differs between versions,
causing the `setSize` assertion to fail.

## GDB Backtrace (key frames)
```
cv::setSize → cv::filter2D → coin_lio::ImageProcessor::removeLines
```
The crash happens in `libopencv_imgproc.so.4.2.0` even though the code was compiled against 4.5.

## Fix 1: LD_PRELOAD in Launch File
Force-load the correct OpenCV 4.5 shared libs before anything else:
```xml
<node pkg="coin_lio" type="coin_lio_mapping" name="laserMapping" output="screen">
  <env name="LD_PRELOAD" value="/usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.5 /usr/lib/aarch64-linux-gnu/libopencv_core.so.4.5"/>
</node>
```

## Fix 2: Link Order in CMakeLists.txt
Link OpenCV BEFORE catkin to prevent cv_bridge from injecting 4.2:
```cmake
# Link OpenCV before catkin to prevent cv_bridge pulling in OpenCV 4.2
add_executable(coin_lio_mapping src/laserMapping.cpp)
target_link_libraries(coin_lio_mapping ${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
```

## Affected Files
- `/home/dev/coinlio_ws/src/coin-lio/launch/mapping_ouster.launch` (LD_PRELOAD)
- `/home/dev/coinlio_ws/src/coin-lio/CMakeLists.txt` (link order)

## When This Happens
- Any ROS Noetic package using OpenCV on Jetson where system OpenCV != 4.2
- Jetson Orin NX ships with OpenCV 4.5, ROS Noetic cv_bridge was built against 4.2
- Symptom: crash in any cv:: function that receives a cv::Mat from a different ABI version

## Prevention
1. Always check for multiple OpenCV versions: `ldconfig -p | grep libopencv`
2. If both 4.2 and 4.5 exist, use LD_PRELOAD for the system version
3. Link order matters: `${OpenCV_LIBRARIES}` before `${catkin_LIBRARIES}`

# SophusConfig.cmake — installed by FAST-LIVO2 Docker build (Sophus commit a621ff2e)
#
# Old Sophus only generates SophusConfig.cmake in the build directory, which is deleted
# after install. This hand-authored config file is installed at build time to:
#   /usr/local/lib/cmake/Sophus/SophusConfig.cmake
# so that downstream FIND_PACKAGE(Sophus) calls from vikit_common and fast_livo succeed.
#
# Headers installed to: /usr/local/include/sophus/
# Library installed to: /usr/local/lib/libSophus.so

set(Sophus_FOUND TRUE)

# Header search path — includes the parent of sophus/ so that
# #include <sophus/so3.h> etc. resolve correctly.
set(Sophus_INCLUDE_DIR  /usr/local/include /usr/include/eigen3)
set(Sophus_INCLUDE_DIRS /usr/local/include /usr/include/eigen3)

set(Sophus_LIBRARIES     /usr/local/lib/libSophus.so)
set(Sophus_LIBRARY       /usr/local/lib/libSophus.so)
set(Sophus_LIBRARY_DIR   /usr/local/lib)
set(Sophus_LIBRARY_DIRS  /usr/local/lib)

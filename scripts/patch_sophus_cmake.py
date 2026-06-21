#!/usr/bin/env python3
"""
Patch old Sophus CMakeLists.txt (commit a621ff) for CMake 3.x compatibility.

Problem: GET_TARGET_PROPERTY(LOCATION) is disallowed by CMake policy CMP0026
in CMake 3.x+. The offending line:
    GET_TARGET_PROPERTY( FULL_LIBRARY_NAME ${PROJECT_NAME} LOCATION )

Fix: Replace with an explicit SET that uses the known output name.
"""
import sys

cmake_file = '/tmp/sophus_src/CMakeLists.txt'

with open(cmake_file, 'r') as fh:
    content = fh.read()

OLD = 'GET_TARGET_PROPERTY( FULL_LIBRARY_NAME ${PROJECT_NAME} LOCATION )'
NEW = 'SET( FULL_LIBRARY_NAME ${CMAKE_BINARY_DIR}/libSophus.so )'

if OLD not in content:
    print(f'WARNING: expected pattern not found in {cmake_file}')
    print('File may already be patched or has unexpected content.')
    sys.exit(0)

patched = content.replace(OLD, NEW)

with open(cmake_file, 'w') as fh:
    fh.write(patched)

print(f'Patched {cmake_file}: replaced GET_TARGET_PROPERTY with SET for CMake 3.x compat')

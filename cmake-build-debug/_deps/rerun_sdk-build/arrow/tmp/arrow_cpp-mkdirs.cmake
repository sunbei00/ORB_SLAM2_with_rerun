# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/root/ORB_SLAM2_with_rerun/cmake-build-debug/_deps/rerun_sdk-build/arrow/src/arrow_cpp")
  file(MAKE_DIRECTORY "/root/ORB_SLAM2_with_rerun/cmake-build-debug/_deps/rerun_sdk-build/arrow/src/arrow_cpp")
endif()
file(MAKE_DIRECTORY
  "/root/ORB_SLAM2_with_rerun/cmake-build-debug/_deps/rerun_sdk-build/arrow/src/arrow_cpp-build"
  "/root/ORB_SLAM2_with_rerun/cmake-build-debug/_deps/rerun_sdk-build/arrow"
  "/root/ORB_SLAM2_with_rerun/cmake-build-debug/_deps/rerun_sdk-build/arrow/tmp"
  "/root/ORB_SLAM2_with_rerun/cmake-build-debug/_deps/rerun_sdk-build/arrow/src/arrow_cpp-stamp"
  "/root/ORB_SLAM2_with_rerun/cmake-build-debug/_deps/rerun_sdk-build/arrow/src"
  "/root/ORB_SLAM2_with_rerun/cmake-build-debug/_deps/rerun_sdk-build/arrow/src/arrow_cpp-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/root/ORB_SLAM2_with_rerun/cmake-build-debug/_deps/rerun_sdk-build/arrow/src/arrow_cpp-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/root/ORB_SLAM2_with_rerun/cmake-build-debug/_deps/rerun_sdk-build/arrow/src/arrow_cpp-stamp${cfgdir}") # cfgdir has leading slash
endif()

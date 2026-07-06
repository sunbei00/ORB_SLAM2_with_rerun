# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/root/ORB_SLAM2_with_rerun/cmake-build-debug/_deps/rerun_sdk-src")
  file(MAKE_DIRECTORY "/root/ORB_SLAM2_with_rerun/cmake-build-debug/_deps/rerun_sdk-src")
endif()
file(MAKE_DIRECTORY
  "/root/ORB_SLAM2_with_rerun/cmake-build-debug/_deps/rerun_sdk-build"
  "/root/ORB_SLAM2_with_rerun/cmake-build-debug/_deps/rerun_sdk-subbuild/rerun_sdk-populate-prefix"
  "/root/ORB_SLAM2_with_rerun/cmake-build-debug/_deps/rerun_sdk-subbuild/rerun_sdk-populate-prefix/tmp"
  "/root/ORB_SLAM2_with_rerun/cmake-build-debug/_deps/rerun_sdk-subbuild/rerun_sdk-populate-prefix/src/rerun_sdk-populate-stamp"
  "/root/ORB_SLAM2_with_rerun/cmake-build-debug/_deps/rerun_sdk-subbuild/rerun_sdk-populate-prefix/src"
  "/root/ORB_SLAM2_with_rerun/cmake-build-debug/_deps/rerun_sdk-subbuild/rerun_sdk-populate-prefix/src/rerun_sdk-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/root/ORB_SLAM2_with_rerun/cmake-build-debug/_deps/rerun_sdk-subbuild/rerun_sdk-populate-prefix/src/rerun_sdk-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/root/ORB_SLAM2_with_rerun/cmake-build-debug/_deps/rerun_sdk-subbuild/rerun_sdk-populate-prefix/src/rerun_sdk-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()

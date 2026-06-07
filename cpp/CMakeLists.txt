# flexiv_control Tier-B real-time daemon.
# Community project, NOT affiliated with Flexiv Robotics.
#
# This builds `rt_server`, the optional 1 kHz host-side control loop. It is NOT
# required for Tier-A (the Python NRT stack). Build it only when you need a true
# 1 kHz host loop. See cpp/README.md for prerequisites and the exact, version-
# matched RDK package name.

cmake_minimum_required(VERSION 3.16)
project(flexiv_control_rt LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-O2 -Wall -Wextra)

# --- Flexiv RDK --------------------------------------------------------------
# Provide the RDK however your install documents it. Most commonly:
#   find_package(flexiv_rdk REQUIRED)         # if you installed the CMake config
# or point CMAKE_PREFIX_PATH at the unpacked RDK install tree:
#   cmake -DCMAKE_PREFIX_PATH=/opt/flexiv_rdk/install ..
# VERIFY: the imported target name differs by RDK version
#   (e.g. flexiv::flexiv_rdk  vs  flexiv_rdk::flexiv_rdk).
find_package(flexiv_rdk REQUIRED)

# --- nlohmann/json (header-only) --------------------------------------------
# Option A (preferred on Ubuntu): sudo apt install nlohmann-json3-dev
# Option B: vendor single header to cpp/third_party/nlohmann/json.hpp
# Option C: FetchContent (needs network at configure time).
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlohmann/json.hpp")
    add_library(nlohmann_json INTERFACE)
    target_include_directories(nlohmann_json INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
    add_library(nlohmann_json::nlohmann_json ALIAS nlohmann_json)
  else()
    message(FATAL_ERROR
      "nlohmann/json not found. Install nlohmann-json3-dev, vendor the single "
      "header to cpp/third_party/nlohmann/json.hpp, or enable FetchContent.")
  endif()
endif()

add_executable(rt_server src/rt_server.cpp)

target_link_libraries(rt_server
  PRIVATE
    flexiv::flexiv_rdk          # VERIFY target name against your RDK version
    nlohmann_json::nlohmann_json
    pthread
)

install(TARGETS rt_server RUNTIME DESTINATION bin)

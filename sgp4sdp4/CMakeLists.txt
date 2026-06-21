# CMakeLists.txt for sgp4sdp4 library
# cmake_minimum_required + project let this build standalone (cmake -S
# sgp4sdp4) on modern CMake (4.x errors without them); needed by CI, which
# builds and installs the static lib before the main project.
cmake_minimum_required(VERSION 3.10)
project(sgp4sdp4 C)
set(SGP4SDP4_SOURCES
    sgp4sdp4.c
    sgp_in.c
    sgp_math.c
    sgp_obs.c
    sgp_time.c
    solar.c
)

add_library(sgp4sdp4 STATIC ${SGP4SDP4_SOURCES})
target_include_directories(sgp4sdp4 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})


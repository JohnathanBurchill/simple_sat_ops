# CMakeLists.txt for sgp4sdp4 library
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


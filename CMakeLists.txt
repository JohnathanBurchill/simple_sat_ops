cmake_minimum_required(VERSION 3.10)
project(RigRotatorControl)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g")

# Add the sgp4sdp4 subdirectory
add_subdirectory(sgp4sdp4)

# Locate Hamlib headers and library
find_path(HAMLIB_INCLUDE_DIR hamlib/rig.h 
    PATHS 
    /usr/local/include 
    /usr/include
)

find_library(HAMLIB_LIBRARY libhamlib.dylib 
    PATHS 
    /usr/local/lib 
    /usr/lib
)

# Debugging output
message(STATUS "Hamlib include dir: ${HAMLIB_INCLUDE_DIR}")
message(STATUS "Hamlib library: ${HAMLIB_LIBRARY}")

# Check if Hamlib was found
if (NOT HAMLIB_INCLUDE_DIR OR NOT HAMLIB_LIBRARY)
    message(FATAL_ERROR "Hamlib not found. Please install Hamlib or specify its location.")
endif()

# Add the main executable
add_executable(rig_rotator_control main.c)

# Include directories for sgp4sdp4 and Hamlib
target_include_directories(rig_rotator_control PRIVATE
    ${CMAKE_SOURCE_DIR}/sgp4sdp4
    ${HAMLIB_INCLUDE_DIR}
)

# Link the sgp4sdp4 library and Hamlib
target_link_libraries(rig_rotator_control PRIVATE sgp4sdp4 ${HAMLIB_LIBRARY} m curl)

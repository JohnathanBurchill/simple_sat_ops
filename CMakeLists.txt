#
#    Simple Satellite Operations  CMakeLists.txt
#
#    Copyright (C) 2025  Johnathan K Burchill
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <https://www.gnu.org/licenses/>.
#

cmake_minimum_required(VERSION 3.10)
project(simple_sat_ops)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g -fsanitize=address")

find_package(PkgConfig REQUIRED)
pkg_check_modules(NCURSES REQUIRED ncurses)

# Add the main executable
add_executable(simple_sat_ops main.c prediction.c radio.c antenna_rotator.c audio.c)
# target_compile_definitions(simple_sat_ops PUBLIC -DQOL)

# Link the sgp4sdp4 library
target_link_libraries(simple_sat_ops PRIVATE sgp4sdp4 ${NCURSES_LIBRARIES} m asound pthread) 

# Next in queue tool
add_executable(next_in_queue next_in_queue.c prediction.c satellite_status.c)
target_link_libraries(next_in_queue PRIVATE sgp4sdp4 m)

# Lifetime tool
add_executable(lifetime lifetime.c prediction.c)
target_link_libraries(lifetime PRIVATE sgp4sdp4 m)

install(TARGETS simple_sat_ops next_in_queue lifetime DESTINATION $ENV{HOME}/bin)

install(PROGRAMS upcoming_amateur_satellite_passes.sh DESTINATION $ENV{HOME}/bin)


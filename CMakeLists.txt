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
pkg_check_modules(OPENSSL REQUIRED libcrypto)

# Add the main executable
add_executable(simple_sat_ops main.c prediction.c radio.c antenna_rotator.c audio.c telemetry.c oem.c)
# target_compile_definitions(simple_sat_ops PUBLIC -DQOL)

# Link the sgp4sdp4 library
target_link_libraries(simple_sat_ops PRIVATE sgp4sdp4 ${NCURSES_LIBRARIES} m asound pthread)

# Next in queue tool
add_executable(next_in_queue next_in_queue.c prediction.c satellite_status.c oem.c)
target_link_libraries(next_in_queue PRIVATE sgp4sdp4 m)

# Lifetime tool
add_executable(lifetime lifetime.c prediction.c oem.c)
target_link_libraries(lifetime PRIVATE sgp4sdp4 m)

# TX test tone tool
add_executable(tx_tone tx_tone.c radio.c audio.c)
target_link_libraries(tx_tone PRIVATE m asound)

# Uplink test tool: builds CSP + AX100 frame, optionally modulates to WAV
add_executable(uplink_test uplink_test.c csp.c golay24.c ax100.c rs.c hmac_keyfile.c modem.c)
target_include_directories(uplink_test PRIVATE ${OPENSSL_INCLUDE_DIRS})
target_link_libraries(uplink_test PRIVATE ${OPENSSL_LIBRARIES} m)

# RS self-test: validates rs.c against reed_solomon_ccsds known vectors
add_executable(rs_selftest utils/rs_selftest.c rs.c)
target_include_directories(rs_selftest PRIVATE ${CMAKE_SOURCE_DIR})

# FM-audio preview tool: baseband WAV -> audible FM-modulated WAV
add_executable(fm_preview utils/fm_preview.c utils/wav_read.c modem.c)
target_include_directories(fm_preview PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(fm_preview PRIVATE m)

# Single-shot frame transmitter: CSP+AX100 -> PCM -> CI-V PTT + ALSA playback
add_executable(tx_frame utils/tx_frame.c csp.c golay24.c ax100.c rs.c hmac_keyfile.c modem.c radio.c)
target_include_directories(tx_frame PRIVATE ${CMAKE_SOURCE_DIR} ${OPENSSL_INCLUDE_DIRS})
target_link_libraries(tx_frame PRIVATE ${OPENSSL_LIBRARIES} m asound)

# Offline AX100 frame decoder: WAV/RAW -> demod -> ax100_unframe -> CSP
add_executable(rx_decode utils/rx_decode.c utils/wav_read.c
               modem.c ax100.c rs.c golay24.c csp.c hmac_keyfile.c)
target_include_directories(rx_decode PRIVATE ${CMAKE_SOURCE_DIR} ${OPENSSL_INCLUDE_DIRS})
target_link_libraries(rx_decode PRIVATE ${OPENSSL_LIBRARIES} m)

install(TARGETS simple_sat_ops next_in_queue lifetime tx_tone uplink_test fm_preview tx_frame rx_decode rs_selftest DESTINATION $ENV{HOME}/bin)

install(PROGRAMS scripts/upcoming_amateur_satellite_passes.sh DESTINATION $ENV{HOME}/bin)
install(PROGRAMS scripts/pcm2wav.sh DESTINATION $ENV{HOME}/bin)


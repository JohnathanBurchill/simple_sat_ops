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

# Mac builds are intended only for radio_ctl + WAV-only / pure-DSP tools so
# the FT-991A CAT path can be exercised without a Linux box. ALSA, ncurses,
# and SGP4SDP4 are all skipped if not present; the targets that need them
# are conditionally added below.
find_package(PkgConfig REQUIRED)
pkg_check_modules(NCURSES ncurses)
pkg_check_modules(ASOUND alsa)
pkg_check_modules(OPENSSL libcrypto)
find_library(SGP4SDP4_LIB sgp4sdp4)

# Optional: USRP B210 backend (requires UHD). When OFF, radio_usrp_b210.c
# is still compiled but its init() returns RADIO_NOT_IMPLEMENTED so the
# binary builds without UHD.
option(WITH_USRP_B210 "Build USRP B210 backend (requires UHD)" OFF)

# Sources shared by every binary that links the radio dispatcher.
set(RADIO_SOURCES radio.c radio_icom_civ.c radio_yaesu_cat.c radio_usrp_b210.c)
if (WITH_USRP_B210)
    add_compile_definitions(WITH_USRP_B210)
endif()

# Built-target accumulator. Every successfully-added target is appended
# here so the `install` line at the bottom doesn't reference targets the
# host couldn't build.
set(SSO_TARGETS)

# ----- Always buildable (no external libs) ----------------------------------

# Radio control / CAT verification tool. Pure radio dispatcher; no audio,
# no SGP4, no ncurses. Builds on macOS for FT-991A bring-up off the Linux
# ground-station box.
add_executable(radio_ctl utils/radio_ctl.c radio_device_store.c ${RADIO_SOURCES})
target_include_directories(radio_ctl PRIVATE ${CMAKE_SOURCE_DIR})
target_compile_definitions(radio_ctl PRIVATE QOL)
target_link_libraries(radio_ctl PRIVATE m)
list(APPEND SSO_TARGETS radio_ctl)

# RS self-test: validates rs.c against reed_solomon_ccsds known vectors
add_executable(rs_selftest utils/rs_selftest.c rs.c)
target_include_directories(rs_selftest PRIVATE ${CMAKE_SOURCE_DIR})
list(APPEND SSO_TARGETS rs_selftest)

# FM-audio preview tool: baseband WAV -> audible FM-modulated WAV
add_executable(fm_preview utils/fm_preview.c utils/wav_read.c modem.c)
target_include_directories(fm_preview PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(fm_preview PRIVATE m)
list(APPEND SSO_TARGETS fm_preview)

# ----- Optional: OpenSSL-dependent (libcrypto) ------------------------------

if (OPENSSL_FOUND)
    # Uplink test tool: builds CSP + AX100 frame, optionally modulates to WAV
    add_executable(uplink_test uplink_test.c csp.c golay24.c ax100.c rs.c hmac_keyfile.c modem.c)
    target_include_directories(uplink_test PRIVATE ${OPENSSL_INCLUDE_DIRS})
    target_link_directories(uplink_test PRIVATE ${OPENSSL_LIBRARY_DIRS})
    target_link_libraries(uplink_test PRIVATE ${OPENSSL_LIBRARIES} m)
    list(APPEND SSO_TARGETS uplink_test)

    # Offline AX100 frame decoder: WAV/RAW -> demod -> ax100_unframe -> CSP
    add_executable(rx_decode utils/rx_decode.c utils/wav_read.c
                   modem.c ax100.c rs.c golay24.c csp.c hmac_keyfile.c)
    target_include_directories(rx_decode PRIVATE ${CMAKE_SOURCE_DIR} ${OPENSSL_INCLUDE_DIRS})
    target_link_directories(rx_decode PRIVATE ${OPENSSL_LIBRARY_DIRS})
    target_link_libraries(rx_decode PRIVATE ${OPENSSL_LIBRARIES} m)
    list(APPEND SSO_TARGETS rx_decode)
else()
    message(STATUS "libcrypto not found; uplink_test and rx_decode will not be built.")
endif()

# ----- Optional: SGP4SDP4-dependent -----------------------------------------

if (SGP4SDP4_LIB)
    # Next-pass scheduling tool
    add_executable(next_in_queue next_in_queue.c prediction.c satellite_status.c oem.c)
    target_link_libraries(next_in_queue PRIVATE ${SGP4SDP4_LIB} m)
    list(APPEND SSO_TARGETS next_in_queue)

    # Toy orbit-decay estimator
    add_executable(lifetime lifetime.c prediction.c oem.c)
    target_link_libraries(lifetime PRIVATE ${SGP4SDP4_LIB} m)
    list(APPEND SSO_TARGETS lifetime)
else()
    message(STATUS "libsgp4sdp4 not found; next_in_queue, lifetime, simple_sat_ops will not be built.")
endif()

# ----- Optional: ALSA-dependent ---------------------------------------------

if (ASOUND_FOUND)
    # TX test tone tool
    add_executable(tx_tone tx_tone.c ${RADIO_SOURCES} radio_device_store.c audio.c)
    target_link_directories(tx_tone PRIVATE ${ASOUND_LIBRARY_DIRS})
    target_link_libraries(tx_tone PRIVATE m ${ASOUND_LIBRARIES})
    list(APPEND SSO_TARGETS tx_tone)

    # Full-band white noise — TX/RX response characterisation
    add_executable(tx_white_noise tx_white_noise.c ${RADIO_SOURCES} radio_device_store.c audio.c)
    target_link_directories(tx_white_noise PRIVATE ${ASOUND_LIBRARY_DIRS})
    target_link_libraries(tx_white_noise PRIVATE m ${ASOUND_LIBRARIES})
    list(APPEND SSO_TARGETS tx_white_noise)

    if (OPENSSL_FOUND)
        # Single-shot frame transmitter: CSP+AX100 -> PCM -> CI-V PTT + ALSA playback
        add_executable(tx_frame utils/tx_frame.c csp.c golay24.c ax100.c rs.c hmac_keyfile.c modem.c radio_device_store.c ${RADIO_SOURCES})
        target_include_directories(tx_frame PRIVATE ${CMAKE_SOURCE_DIR} ${OPENSSL_INCLUDE_DIRS})
        target_link_directories(tx_frame PRIVATE ${OPENSSL_LIBRARY_DIRS} ${ASOUND_LIBRARY_DIRS})
        target_link_libraries(tx_frame PRIVATE ${OPENSSL_LIBRARIES} m ${ASOUND_LIBRARIES})
        list(APPEND SSO_TARGETS tx_frame)
    endif()
else()
    message(STATUS "libasound not found; tx_tone, tx_frame will not be built.")
endif()

# ----- Optional: full main UI (needs ALSA + ncurses + SGP4) -----------------

if (ASOUND_FOUND AND NCURSES_FOUND AND SGP4SDP4_LIB)
    add_executable(simple_sat_ops main.c prediction.c ${RADIO_SOURCES} antenna_rotator.c audio.c telemetry.c oem.c)
    target_link_directories(simple_sat_ops PRIVATE ${ASOUND_LIBRARY_DIRS} ${NCURSES_LIBRARY_DIRS})
    target_link_libraries(simple_sat_ops PRIVATE ${SGP4SDP4_LIB} ${NCURSES_LIBRARIES} m ${ASOUND_LIBRARIES} pthread)
    list(APPEND SSO_TARGETS simple_sat_ops)
else()
    message(STATUS "simple_sat_ops requires ALSA + ncurses + sgp4sdp4; skipping.")
endif()

# ----- Install --------------------------------------------------------------

install(TARGETS ${SSO_TARGETS} DESTINATION $ENV{HOME}/bin)
install(PROGRAMS scripts/upcoming_amateur_satellite_passes.sh DESTINATION $ENV{HOME}/bin)
install(PROGRAMS scripts/pcm2wav.sh DESTINATION $ENV{HOME}/bin)

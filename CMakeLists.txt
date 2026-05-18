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

# Catch buffer-truncation bugs on the dev host so they don't slip to
# the RAO Linux build. clang and gcc both accept -Wformat-truncation;
# unknown -W flags become a single warning and the build still proceeds,
# so unconditional add is safe across host compilers.
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wformat-truncation -Wformat-overflow")

# Sources live under src/<area>/. Every #include in the codebase is the
# bare form `#include "foo.h"`, so we register every area as an include
# directory once here and avoid editing thousands of include lines. New
# areas: add the path below and `git mv` files into it.
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/dsp
    ${CMAKE_SOURCE_DIR}/src/proto
    ${CMAKE_SOURCE_DIR}/src/beacon
    ${CMAKE_SOURCE_DIR}/src/orbit
    ${CMAKE_SOURCE_DIR}/src/hw
    ${CMAKE_SOURCE_DIR}/src/pipeline
    ${CMAKE_SOURCE_DIR}/src/ipc
    ${CMAKE_SOURCE_DIR}/src/db
)

# SDR-only build. The CAT-radio dispatcher (radio.c + radio_*.c) and the
# ALSA-bound tools (tx_tone, tx_white_noise, tx_frame, rx_capture,
# rx_live) have been retired. Hardware control of the RF chain now goes
# through the B210 (libuhd) directly; operator coordination is in-band
# over the sso_ipc socket (see src/ipc/sso_*.c).
find_package(PkgConfig REQUIRED)
pkg_check_modules(NCURSES ncurses)
pkg_check_modules(OPENSSL libcrypto)
pkg_check_modules(SQLITE3 sqlite3)
find_library(SGP4SDP4_LIB sgp4sdp4)

# packet_db is a shared dependency of every receiver: rx_decode,
# rx_replay, b210_rx_tx all tap emit_frame and write each decoded
# packet into a project-wide SQLite store.
if (NOT SQLITE3_FOUND)
    message(WARNING "sqlite3 not found via pkg-config; packet_db will "
                    "compile as a no-op stub. Install sqlite3 (`brew "
                    "install sqlite` on macOS) for full DB support.")
endif()
function(target_link_packet_db target)
    target_sources(${target} PRIVATE src/db/packet_db.c)
    if (SQLITE3_FOUND)
        target_compile_definitions(${target} PRIVATE WITH_SQLITE3)
        target_include_directories(${target} PRIVATE ${SQLITE3_INCLUDE_DIRS})
        target_link_directories(${target} PRIVATE ${SQLITE3_LIBRARY_DIRS})
        target_link_libraries(${target} PRIVATE ${SQLITE3_LIBRARIES})
    endif()
endfunction()

# UHD is required for the B210-side tools (b210_rx_capture, b210_rx_tx,
# tx_frame_sdr). Mac dev hosts that don't have UHD can skip those with
# `cmake .. -DWITH_USRP_B210=OFF`.
option(WITH_USRP_B210 "Build USRP B210 tools (requires UHD)" ON)
if (WITH_USRP_B210)
    pkg_check_modules(UHD uhd)
    if (NOT UHD_FOUND)
        message(FATAL_ERROR "WITH_USRP_B210=ON but UHD not found via pkg-config. "
                            "Install UHD (e.g. `brew install uhd` on macOS) and retry, "
                            "or rerun with -DWITH_USRP_B210=OFF.")
    endif()
endif()

# sso_core: small set of helpers used by every operator/viewer-aware
# tool plus the audit log. Pure POSIX; builds on every host. Pulled
# in via target_link_sso_core() rather than a SHARED lib so the build
# stays single-binary-per-target.
set(SSO_CORE_SOURCES
    src/ipc/sso_paths.c
    src/ipc/sso_audit.c
    src/ipc/sso_ipc_paths.c
    src/ipc/sso_ipc.c
    src/ipc/sso_operator.c)
function(target_link_sso_core target)
    target_sources(${target} PRIVATE ${SSO_CORE_SOURCES})
endfunction()

# Built-target accumulator. Every successfully-added target is appended
# here so the `install` line at the bottom doesn't reference targets the
# host couldn't build.
set(SSO_TARGETS)

# ----- Always buildable (no external libs) ----------------------------------

# Unit tests live in unit_tests/. Each *_selftest.c emits TAP
# (Test Anything Protocol) via unit_tests/tap.h, so the runner can
# parse per-assertion pass/fail without each test reinventing the
# wheel. tap.h is shared via the include path below.
set(UNIT_TESTS_INCLUDE ${CMAKE_SOURCE_DIR}/unit_tests)

# RS self-test: validates rs.c against reed_solomon_ccsds known vectors
add_executable(rs_selftest unit_tests/rs_selftest.c src/proto/rs.c)
target_include_directories(rs_selftest PRIVATE ${UNIT_TESTS_INCLUDE})
list(APPEND SSO_TARGETS rs_selftest)

# Antenna rotator self-test: covers the pure math helpers in
# antenna_rotator.c (wrap_to_pm180, accumulate_unwrapped,
# home_unwrapped_target, to_mech_coords). No serial/UHD/ncurses deps;
# builds on every host.
add_executable(antenna_rotator_selftest
               unit_tests/antenna_rotator_selftest.c src/hw/antenna_rotator.c)
target_include_directories(antenna_rotator_selftest PRIVATE ${UNIT_TESTS_INCLUDE})
target_link_libraries(antenna_rotator_selftest PRIVATE m)
list(APPEND SSO_TARGETS antenna_rotator_selftest)

# TLE CSV self-test: round-trips Celestrak OMM CSV through
# tle_path_resolve and audits the resulting 3-line TLE. Pure POSIX,
# no external library deps.
add_executable(tle_csv_selftest
               unit_tests/tle_csv_selftest.c src/orbit/tle_csv.c)
target_include_directories(tle_csv_selftest PRIVATE ${UNIT_TESTS_INCLUDE})
target_link_libraries(tle_csv_selftest PRIVATE m)
list(APPEND SSO_TARGETS tle_csv_selftest)

# Curses test runner: walks the *_selftest binaries built above (and
# any added later in this file), parses TAP from their stdout, and
# renders a collapsible group view. Requires ncurses; if not present
# the runner just isn't built.
if (NCURSES_FOUND)
    add_executable(unit_test_runner unit_tests/runner.c)
    target_include_directories(unit_test_runner PRIVATE
        ${UNIT_TESTS_INCLUDE} ${NCURSES_INCLUDE_DIRS})
    target_link_directories(unit_test_runner PRIVATE ${NCURSES_LIBRARY_DIRS})
    target_link_libraries(unit_test_runner PRIVATE ${NCURSES_LIBRARIES})
    list(APPEND SSO_TARGETS unit_test_runner)
endif()

# FM-audio preview tool: baseband WAV -> audible FM-modulated WAV
add_executable(fm_preview utils/fm_preview.c utils/wav_read.c
               src/dsp/modem.c)
target_link_libraries(fm_preview PRIVATE m)
list(APPEND SSO_TARGETS fm_preview)

# Beacon-cadence detector for post-pass triage of FM-demoded WAVs.
add_executable(beacon_detect utils/beacon_detect.c utils/wav_read.c
               src/dsp/biquad.c src/dsp/monitor_squelch.c)
target_link_libraries(beacon_detect PRIVATE m)
list(APPEND SSO_TARGETS beacon_detect)

# SatNOGS-style waterfall renderer: raw int16 IQ → viridis PNG.
# Self-contained — no libpng or libz dependency (stored-DEFLATE PNG +
# inlined radix-2 FFT). simple_sat_ops invokes it for `:spectrum N` and
# end-of-pass renders when an IQ sidecar is available.
add_executable(gen_waterfall utils/gen_waterfall.c)
target_link_libraries(gen_waterfall PRIVATE m)
list(APPEND SSO_TARGETS gen_waterfall)

# Selftest for the IQ-domain demods (modem_iq.c, modem_viterbi.c).
# Synthesizes CPFSK + optional AWGN, round-trips through both chains,
# and A/B-checks against modem_pcm16_to_bits on the FM-discriminated
# equivalent.
add_executable(modem_iq_selftest unit_tests/modem_iq_selftest.c
               src/dsp/modem.c src/dsp/modem_fsk.c
               src/dsp/modem_iq.c src/dsp/modem_viterbi.c)
target_include_directories(modem_iq_selftest PRIVATE ${UNIT_TESTS_INCLUDE})
target_link_libraries(modem_iq_selftest PRIVATE m)
list(APPEND SSO_TARGETS modem_iq_selftest)

# Software-Doppler NCO selftest. Runs without UHD — the NCO is the
# DSP core, extracted so it can be exercised on synthesised IQ.
add_executable(sw_nco_selftest unit_tests/sw_nco_selftest.c
               src/dsp/sw_nco.c)
target_include_directories(sw_nco_selftest PRIVATE ${UNIT_TESTS_INCLUDE})
target_link_libraries(sw_nco_selftest PRIVATE m)
list(APPEND SSO_TARGETS sw_nco_selftest)

# ----- Optional: OpenSSL-dependent (libcrypto) ------------------------------

if (OPENSSL_FOUND)
    # Uplink test tool: builds CSP + AX100 frame, optionally modulates to WAV
    add_executable(uplink_test apps/uplink_test.c
                   src/proto/csp.c src/proto/golay24.c src/proto/ax100.c
                   src/proto/rs.c src/proto/hmac_keyfile.c src/dsp/modem.c)
    target_include_directories(uplink_test PRIVATE ${OPENSSL_INCLUDE_DIRS})
    target_link_directories(uplink_test PRIVATE ${OPENSSL_LIBRARY_DIRS})
    target_link_libraries(uplink_test PRIVATE ${OPENSSL_LIBRARIES} m)
    list(APPEND SSO_TARGETS uplink_test)

    # Offline AX100 frame decoder: WAV/RAW -> demod -> ax100_unframe -> CSP
    add_executable(rx_decode utils/rx_decode.c utils/wav_read.c
                   src/dsp/modem.c src/dsp/modem_fsk.c
                   src/dsp/modem_iq.c src/dsp/modem_viterbi.c
                   src/proto/ax100.c src/proto/rs.c src/proto/golay24.c
                   src/proto/csp.c src/proto/hmac_keyfile.c
                   src/beacon/beacon_cts1.c src/pipeline/decode_loop.c)
    target_include_directories(rx_decode PRIVATE ${OPENSSL_INCLUDE_DIRS})
    target_link_directories(rx_decode PRIVATE ${OPENSSL_LIBRARY_DIRS})
    target_link_libraries(rx_decode PRIVATE ${OPENSSL_LIBRARIES} m)
    target_link_packet_db(rx_decode)
    target_link_sso_core(rx_decode)
    list(APPEND SSO_TARGETS rx_decode)

    # Non-interactive query CLI over the packet DB.
    add_executable(packet_query utils/packet_query.c)
    target_include_directories(packet_query PRIVATE ${OPENSSL_INCLUDE_DIRS})
    target_link_directories(packet_query PRIVATE ${OPENSSL_LIBRARY_DIRS})
    target_link_libraries(packet_query PRIVATE ${OPENSSL_LIBRARIES} m)
    target_link_packet_db(packet_query)
    target_link_sso_core(packet_query)
    list(APPEND SSO_TARGETS packet_query)

    # Curses TUI sibling of packet_query.
    if (NCURSES_FOUND)
        add_executable(packet_browser utils/packet_browser.c)
        target_compile_definitions(packet_browser PRIVATE
            PACKET_BROWSER_HAVE_NCURSES)
        target_include_directories(packet_browser PRIVATE
            ${OPENSSL_INCLUDE_DIRS} ${NCURSES_INCLUDE_DIRS})
        target_link_directories(packet_browser PRIVATE
            ${OPENSSL_LIBRARY_DIRS} ${NCURSES_LIBRARY_DIRS})
        target_link_libraries(packet_browser PRIVATE
            ${OPENSSL_LIBRARIES} ${NCURSES_LIBRARIES} m)
        target_link_packet_db(packet_browser)
        target_link_sso_core(packet_browser)
        list(APPEND SSO_TARGETS packet_browser)
    endif()

    # Offline sliding-window decoder for WAV / raw S16_LE files.
    add_executable(rx_replay utils/rx_replay.c utils/wav_read.c
                   src/pipeline/decode_loop.c
                   src/dsp/modem.c src/dsp/modem_fsk.c
                   src/dsp/modem_iq.c src/dsp/modem_viterbi.c
                   src/proto/ax100.c src/proto/rs.c src/proto/golay24.c
                   src/proto/csp.c src/proto/hmac_keyfile.c
                   src/beacon/beacon_cts1.c src/pipeline/rx_tui.c
                   src/orbit/tle_csv.c)
    target_include_directories(rx_replay PRIVATE ${OPENSSL_INCLUDE_DIRS})
    target_link_directories(rx_replay PRIVATE ${OPENSSL_LIBRARY_DIRS})
    target_link_libraries(rx_replay PRIVATE ${OPENSSL_LIBRARIES} m)
    if (NCURSES_FOUND)
        target_compile_definitions(rx_replay PRIVATE RX_TUI_AVAILABLE)
        target_include_directories(rx_replay PRIVATE ${NCURSES_INCLUDE_DIRS})
        target_link_directories(rx_replay PRIVATE ${NCURSES_LIBRARY_DIRS})
        target_link_libraries(rx_replay PRIVATE ${NCURSES_LIBRARIES})
    endif()
    if (SGP4SDP4_LIB)
        target_compile_definitions(rx_replay PRIVATE WITH_SGP4SDP4)
        target_sources(rx_replay PRIVATE src/orbit/prediction.c src/orbit/oem.c)
        target_link_libraries(rx_replay PRIVATE ${SGP4SDP4_LIB})
    endif()
    target_link_packet_db(rx_replay)
    target_link_sso_core(rx_replay)
    list(APPEND SSO_TARGETS rx_replay)

    # AX100 framer self-test: structural audit + round trip across the
    # opts matrix + Golay/RS/HMAC error injection. Needs OpenSSL (HMAC)
    # so it lives in the OPENSSL_FOUND block.
    add_executable(ax100_selftest unit_tests/ax100_selftest.c
                   src/proto/ax100.c src/proto/rs.c
                   src/proto/golay24.c src/proto/hmac_keyfile.c)
    target_include_directories(ax100_selftest PRIVATE
        ${UNIT_TESTS_INCLUDE} ${OPENSSL_INCLUDE_DIRS})
    target_link_directories(ax100_selftest PRIVATE ${OPENSSL_LIBRARY_DIRS})
    target_link_libraries(ax100_selftest PRIVATE ${OPENSSL_LIBRARIES} m)
    list(APPEND SSO_TARGETS ax100_selftest)

    # CTS1 beacon WAV generator
    add_executable(beacon_gen utils/beacon_gen.c
                   src/proto/csp.c src/proto/golay24.c src/proto/ax100.c
                   src/proto/rs.c src/proto/hmac_keyfile.c src/dsp/modem.c)
    target_include_directories(beacon_gen PRIVATE ${OPENSSL_INCLUDE_DIRS})
    target_link_directories(beacon_gen PRIVATE ${OPENSSL_LIBRARY_DIRS})
    target_link_libraries(beacon_gen PRIVATE ${OPENSSL_LIBRARIES} m)
    list(APPEND SSO_TARGETS beacon_gen)

    # SDR single-shot frame transmitter (B210 / libuhd)
    if (WITH_USRP_B210)
        add_executable(tx_frame_sdr utils/tx_frame_sdr.c
                       src/proto/csp.c src/proto/golay24.c src/proto/ax100.c
                       src/proto/rs.c src/proto/hmac_keyfile.c src/dsp/modem.c)
        target_include_directories(tx_frame_sdr PRIVATE
            ${OPENSSL_INCLUDE_DIRS} ${UHD_INCLUDE_DIRS})
        target_link_directories(tx_frame_sdr PRIVATE
            ${OPENSSL_LIBRARY_DIRS} ${UHD_LIBRARY_DIRS})
        target_link_libraries(tx_frame_sdr PRIVATE
            ${OPENSSL_LIBRARIES} ${UHD_LIBRARIES} m)
        target_link_sso_core(tx_frame_sdr)
        list(APPEND SSO_TARGETS tx_frame_sdr)

        # B210 record-only RX capture.
        add_executable(b210_rx_capture utils/b210_rx_capture.c
                       src/dsp/modem.c)
        target_include_directories(b210_rx_capture PRIVATE
            ${UHD_INCLUDE_DIRS})
        target_link_directories(b210_rx_capture PRIVATE
            ${UHD_LIBRARY_DIRS})
        target_link_libraries(b210_rx_capture PRIVATE
            ${UHD_LIBRARIES} m)
        target_link_sso_core(b210_rx_capture)
        list(APPEND SSO_TARGETS b210_rx_capture)

        # Live B210 RX + TX is no longer a separate target — the
        # daemon was folded into simple_sat_ops (FLYUFLS), which now
        # owns the device end-to-end. b210_rx_capture survives for
        # offline IQ recording; tx_frame_sdr survives for offline IQ
        # rendering. b210_rx_tx_core.c links into simple_sat_ops via
        # the WITH_USRP_B210 block below.
    endif()
else()
    message(STATUS "libcrypto not found; uplink_test and rx_decode will not be built.")
endif()

# ----- Optional: SGP4SDP4-dependent -----------------------------------------

if (SGP4SDP4_LIB)
    # Next-pass scheduling tool
    add_executable(next_in_queue apps/next_in_queue.c
                   src/orbit/prediction.c src/beacon/satellite_status.c
                   src/orbit/oem.c src/orbit/tle_csv.c)
    target_link_libraries(next_in_queue PRIVATE ${SGP4SDP4_LIB} m)
    list(APPEND SSO_TARGETS next_in_queue)

    # Toy orbit-decay estimator
    add_executable(lifetime apps/lifetime.c
                   src/orbit/prediction.c src/orbit/oem.c src/orbit/tle_csv.c)
    target_link_libraries(lifetime PRIVATE ${SGP4SDP4_LIB} m)
    list(APPEND SSO_TARGETS lifetime)

    # Prediction self-test: load a frozen fixture TLE, exercise
    # tle_default_path / load_tle / update_satellite_position /
    # update_pass_predictions, and assert physically-plausible ranges.
    # Pulls oem.c + ncurses through prediction.c's transitive deps.
    add_executable(prediction_selftest
                   unit_tests/prediction_selftest.c
                   src/orbit/prediction.c src/orbit/oem.c)
    target_include_directories(prediction_selftest PRIVATE ${UNIT_TESTS_INCLUDE})
    target_link_libraries(prediction_selftest PRIVATE ${SGP4SDP4_LIB} m)
    if (NCURSES_FOUND)
        target_include_directories(prediction_selftest PRIVATE
            ${NCURSES_INCLUDE_DIRS})
        target_link_directories(prediction_selftest PRIVATE
            ${NCURSES_LIBRARY_DIRS})
        target_link_libraries(prediction_selftest PRIVATE
            ${NCURSES_LIBRARIES})
    endif()
    list(APPEND SSO_TARGETS prediction_selftest)
else()
    message(STATUS "libsgp4sdp4 not found; next_in_queue, lifetime, simple_sat_ops will not be built.")
endif()

# ----- Full operator UI (needs ncurses + SGP4 + OpenSSL) --------------------
# simple_sat_ops now owns the live RF chain too: B210 RX + AX100 decode +
# in-process TX bursts. The framing / decode / squelch / HMAC sources are
# linked directly; the UHD-dependent pieces sit behind WITH_USRP_B210.

if (NCURSES_FOUND AND SGP4SDP4_LIB AND OPENSSL_FOUND)
    add_executable(simple_sat_ops apps/main.c
                   src/orbit/prediction.c src/orbit/oem.c
                   src/hw/antenna_rotator.c
                   src/beacon/telemetry.c
                   src/pipeline/decode_loop.c
                   src/dsp/modem.c src/dsp/modem_fsk.c
                   src/dsp/modem_iq.c src/dsp/modem_viterbi.c
                   src/proto/ax100.c src/proto/rs.c
                   src/proto/golay24.c src/proto/csp.c
                   src/proto/hmac_keyfile.c
                   src/beacon/beacon_cts1.c
                   src/orbit/tle_csv.c
                   src/dsp/biquad.c src/dsp/monitor_squelch.c)
    target_include_directories(simple_sat_ops PRIVATE
        ${NCURSES_INCLUDE_DIRS} ${OPENSSL_INCLUDE_DIRS})
    target_link_directories(simple_sat_ops PRIVATE
        ${NCURSES_LIBRARY_DIRS} ${OPENSSL_LIBRARY_DIRS})
    target_link_libraries(simple_sat_ops PRIVATE
        ${SGP4SDP4_LIB} ${NCURSES_LIBRARIES} ${OPENSSL_LIBRARIES} m pthread)
    if (WITH_USRP_B210)
        target_compile_definitions(simple_sat_ops PRIVATE WITH_USRP_B210)
        target_sources(simple_sat_ops PRIVATE
                       src/hw/b210_rx_tx_core.c
                       src/dsp/fir_decim.c src/dsp/sw_nco.c
                       src/pipeline/rx_session.c src/pipeline/tx_burst.c)
        target_include_directories(simple_sat_ops PRIVATE ${UHD_INCLUDE_DIRS})
        target_link_directories(simple_sat_ops PRIVATE ${UHD_LIBRARY_DIRS})
        target_link_libraries(simple_sat_ops PRIVATE ${UHD_LIBRARIES})
    endif()
    target_link_packet_db(simple_sat_ops)
    target_link_sso_core(simple_sat_ops)
    list(APPEND SSO_TARGETS simple_sat_ops)
else()
    message(STATUS "simple_sat_ops requires ncurses + sgp4sdp4 + libcrypto; skipping.")
endif()

# ----- Install --------------------------------------------------------------
#
# Ground-machine deployment goes to /usr/local/bin (system-wide). The
# operator-mode tools need to be on PATH for every account in sso-ops,
# and /usr/local/bin is the canonical location. `make install` requires
# sudo on most setups.

install(TARGETS ${SSO_TARGETS} DESTINATION /usr/local/bin)

file(GLOB SSO_SCRIPTS
    "${CMAKE_SOURCE_DIR}/scripts/*.sh"
    "${CMAKE_SOURCE_DIR}/scripts/*.py")
if (SSO_SCRIPTS)
    install(PROGRAMS ${SSO_SCRIPTS} DESTINATION /usr/local/bin)
endif()

# Sysadmin examples — copied to /usr/local/share/sso/etc/ alongside the
# binaries. Sysadmin then installs them into /etc/ during the one-time
# ground-machine setup. See scripts/sso_setup_root.sh.
install(DIRECTORY etc/ DESTINATION /usr/local/share/sso/etc
        FILES_MATCHING
            PATTERN "*.conf"
            PATTERN "*.rules"
            PATTERN "sso")

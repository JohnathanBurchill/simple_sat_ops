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

# Emit compile_commands.json so clangd (LSP) gets exact per-file flags and
# include paths. Symlink it into the source root once: `ln -sf
# build/compile_commands.json compile_commands.json`. clangd prefers it over
# the hand-maintained compile_flags.txt fallback.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
    ${CMAKE_SOURCE_DIR}/src/cli
    ${CMAKE_SOURCE_DIR}/src/ui
    ${CMAKE_SOURCE_DIR}/src/control
    ${CMAKE_BINARY_DIR}/generated
)

# Bake the git commit into sso_version.h so every tool's -V / --version
# can report which build it is (see cmake/sso_version.h.in). Captured at
# configure time; CMAKE_CONFIGURE_DEPENDS on the git refs below makes a
# commit retrigger configure, so a plain `make` after committing
# regenerates the header. configure_file only rewrites it when the
# contents actually change, so unchanged rebuilds don't recompile.
find_package(Git QUIET)
set(SSO_VERSION_COMMIT "unknown")
set(SSO_VERSION_DATE "")
set(SSO_VERSION_DIRTY "")
if (Git_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} -C ${CMAKE_SOURCE_DIR} rev-parse --short=12 HEAD
        OUTPUT_VARIABLE _sso_commit OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET RESULT_VARIABLE _sso_rc)
    if (_sso_rc EQUAL 0 AND NOT _sso_commit STREQUAL "")
        set(SSO_VERSION_COMMIT "${_sso_commit}")
    endif()
    execute_process(
        COMMAND ${GIT_EXECUTABLE} -C ${CMAKE_SOURCE_DIR} show -s --format=%cs HEAD
        OUTPUT_VARIABLE SSO_VERSION_DATE OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} -C ${CMAKE_SOURCE_DIR} status --porcelain
        OUTPUT_VARIABLE _sso_dirty OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    if (NOT _sso_dirty STREQUAL "")
        set(SSO_VERSION_DIRTY "-dirty")
    endif()
    foreach (_gitfile logs/HEAD index)
        if (EXISTS ${CMAKE_SOURCE_DIR}/.git/${_gitfile})
            set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
                         ${CMAKE_SOURCE_DIR}/.git/${_gitfile})
        endif()
    endforeach()
endif()
configure_file(${CMAKE_SOURCE_DIR}/cmake/sso_version.h.in
               ${CMAKE_BINARY_DIR}/generated/sso_version.h @ONLY)

# SDR-only build. The CAT-radio dispatcher (radio.c + radio_*.c) and the
# ALSA-bound tools (tx_tone, tx_white_noise, tx_frame, rx_capture,
# rx_live) have been retired. Hardware control of the RF chain now goes
# through the B210 (libuhd) directly; operator coordination is in-band
# over the sso_ipc socket (see src/ipc/sso_*.c).
find_package(PkgConfig REQUIRED)
pkg_check_modules(NCURSES ncurses)
pkg_check_modules(OPENSSL libcrypto)
pkg_check_modules(SQLITE3 sqlite3)
pkg_check_modules(RAYLIB raylib)
# libsndfile decodes SatNOGS .ogg (and .wav/.flac) straight to PCM in
# memory, so rx_replay can replay a SatNOGS audio recording. Optional:
# without it, rx_replay still handles .wav/.iq, and .ogg errors with a
# clear hint.
pkg_check_modules(SNDFILE sndfile)
find_library(SGP4SDP4_LIB sgp4sdp4)
# pthread for the sso_audit writer thread + the rx_session worker.
# Used by target_link_sso_core() and the simple_sat_ops link line.
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

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

# RTL-SDR (RX-only) backend. On by default, but SOFT: unlike UHD it
# auto-disables when librtlsdr is absent rather than failing the build,
# so hosts without the library (or without any SDR at all) still
# configure. simple_sat_ops can be built with UHD, RTL-SDR, or both; at
# runtime --sdr-type / auto picks the device. An RTL-SDR cannot
# transmit, so TX is disabled when one is in use.
# libusb: the UHD backend reads a device's USB serial through it to
# pick the right FPGA image for a B2xx clone before opening (UHD's own
# uhd_usrp_find() segfaults on macOS, so we can't use it). UHD depends
# on libusb, so it's normally present wherever UHD is; if it's somehow
# absent, serial-based clone detection just compiles out.
if (WITH_USRP_B210)
    pkg_check_modules(LIBUSB libusb-1.0)
endif()

option(WITH_RTL_SDR "Build the RTL-SDR RX-only backend (needs librtlsdr)" ON)
if (WITH_RTL_SDR)
    pkg_check_modules(RTLSDR librtlsdr)
    if (NOT RTLSDR_FOUND)
        message(STATUS "librtlsdr not found; RTL-SDR backend disabled. "
                       "Install librtlsdr to enable it, or pass "
                       "-DWITH_RTL_SDR=OFF to silence this note.")
        set(WITH_RTL_SDR OFF)
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
    # sso_audit.c spawns a writer pthread to keep the producer side
    # non-blocking. Apple libSystem provides pthread implicitly; Linux
    # needs -lpthread. CMake's Threads::Threads handles both.
    target_link_libraries(${target} PRIVATE Threads::Threads)
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

# Pursuit-tracking self-test: closed-form mock sat tracks exercising
# the whole-pass planner in src/orbit/pursuit.c (no SGP4 / rotator
# dependency). See ~/.claude/plans/let-s-implement-a-decode-dynamic-newt.md.
add_executable(pursuit_selftest
               unit_tests/pursuit_selftest.c src/orbit/pursuit.c)
target_include_directories(pursuit_selftest PRIVATE ${UNIT_TESTS_INCLUDE})
target_link_libraries(pursuit_selftest PRIVATE m)
list(APPEND SSO_TARGETS pursuit_selftest)

# TLE CSV self-test: round-trips Celestrak OMM CSV through
# tle_path_resolve and audits the resulting 3-line TLE. Pure POSIX,
# no external library deps.
add_executable(tle_csv_selftest
               unit_tests/tle_csv_selftest.c src/orbit/tle_csv.c)
target_include_directories(tle_csv_selftest PRIVATE ${UNIT_TESTS_INCLUDE})
target_link_libraries(tle_csv_selftest PRIVATE m)
list(APPEND SSO_TARGETS tle_csv_selftest)

# NovAtel BESTXYZA parser self-test: the team's example log checked
# against their Python field values, plus the GPS-week/seconds epoch,
# the message CRC, and the wrapper / corruption paths. Pure POSIX.
add_executable(bestxyz_selftest
               unit_tests/bestxyz_selftest.c src/orbit/bestxyz.c)
target_include_directories(bestxyz_selftest PRIVATE ${UNIT_TESTS_INCLUDE})
target_link_libraries(bestxyz_selftest PRIVATE m)
list(APPEND SSO_TARGETS bestxyz_selftest)

# Agenda-line self-test: the shared inline-comment rule (agenda_line.c)
# that simple_sat_ops (transmit) and agenda_check (audit) both use to
# split a telecommand from a trailing '#' comment. Pure POSIX.
add_executable(agenda_line_selftest
               unit_tests/agenda_line_selftest.c src/proto/agenda_line.c)
target_include_directories(agenda_line_selftest PRIVATE ${UNIT_TESTS_INCLUDE})
list(APPEND SSO_TARGETS agenda_line_selftest)

# Telecommand-lint self-test: the firmware-derived agenda linter
# (tcmd_lint.c + the generated tcmd_spec.c table) shared by agenda_check
# and simple_sat_ops. Pure POSIX.
add_executable(tcmd_lint_selftest
               unit_tests/tcmd_lint_selftest.c src/proto/tcmd_lint.c
               src/proto/tcmd_spec.c src/proto/agenda_line.c
               src/proto/sso_pseudo.c)
target_include_directories(tcmd_lint_selftest PRIVATE ${UNIT_TESTS_INCLUDE})
list(APPEND SSO_TARGETS tcmd_lint_selftest)

# SSO+ pseudo-command self-test: the expander/registry (sso_pseudo.c) that
# simple_sat_ops expands at TX time, plus a round-trip that the expansion
# lints clean (so it links the linter + spec + agenda_line too). Pure POSIX.
add_executable(sso_pseudo_selftest
               unit_tests/sso_pseudo_selftest.c src/proto/sso_pseudo.c
               src/proto/tcmd_lint.c src/proto/tcmd_spec.c src/proto/agenda_line.c)
target_include_directories(sso_pseudo_selftest PRIVATE ${UNIT_TESTS_INCLUDE})
list(APPEND SSO_TARGETS sso_pseudo_selftest)

# argparse self-test: the dual-mode parse_args() convention
# (src/cli/argparse.h). Header-only, so the test compiles on its own.
# Pure POSIX.
add_executable(argparse_selftest
               unit_tests/argparse_selftest.c)
target_include_directories(argparse_selftest PRIVATE ${UNIT_TESTS_INCLUDE})
list(APPEND SSO_TARGETS argparse_selftest)

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

# agenda_check: TC-file reviewer. Humanizes @tssent=/@tsexec= unix-ms
# values into ISO timestamps (UTC or --local-time) and flags verbatim-
# duplicate command lines. Pure POSIX core. When sgp4sdp4 is present it
# also accepts --tle <file>, propagating the first satellite (via the
# same prediction.c path next_in_queue uses) to append the sub-satellite
# lat/lon/alt at each command's execution time.
add_executable(agenda_check utils/agenda_check.c src/proto/agenda_line.c
               src/proto/tcmd_lint.c src/proto/tcmd_spec.c src/proto/sso_pseudo.c)
if (SGP4SDP4_LIB)
    target_compile_definitions(agenda_check PRIVATE WITH_SGP4SDP4)
    target_sources(agenda_check PRIVATE src/orbit/prediction.c src/orbit/oem.c)
    target_link_libraries(agenda_check PRIVATE ${SGP4SDP4_LIB} m)
endif()
list(APPEND SSO_TARGETS agenda_check)

# Beacon-cadence detector for post-pass triage of FM-demoded WAVs.
add_executable(beacon_detect utils/beacon_detect.c utils/wav_read.c
               src/dsp/biquad.c src/dsp/monitor_squelch.c)
target_link_libraries(beacon_detect PRIVATE m)
list(APPEND SSO_TARGETS beacon_detect)

# Shared FFT / dB / detrend / notch / zoom pipeline. Used by both the
# CLI waterfall renderer (gen_waterfall) and the interactive annotator
# (decode_inspector, which uses it to compute the spectrogram in-process
# and re-colour on the GPU rather than regenerating a PNG every time
# the operator nudges the dB range).
add_library(waterfall_core STATIC utils/waterfall_core.c)
target_include_directories(waterfall_core PUBLIC utils)
target_link_libraries(waterfall_core PUBLIC m)

# SatNOGS-style waterfall renderer: raw int16 IQ → viridis PNG.
# Self-contained — no libpng or libz dependency (stored-DEFLATE PNG +
# inlined radix-2 FFT). simple_sat_ops invokes it for `:spectrum N` and
# end-of-pass renders when an IQ sidecar is available.
add_executable(gen_waterfall utils/gen_waterfall.c src/dsp/sw_nco.c)
target_link_libraries(gen_waterfall PRIVATE waterfall_core m)
if (SNDFILE_FOUND)
    target_compile_definitions(gen_waterfall PRIVATE HAVE_SNDFILE)
    target_include_directories(gen_waterfall PRIVATE ${SNDFILE_INCLUDE_DIRS})
    target_link_directories(gen_waterfall PRIVATE ${SNDFILE_LIBRARY_DIRS})
    target_link_libraries(gen_waterfall PRIVATE ${SNDFILE_LIBRARIES})
endif()
list(APPEND SSO_TARGETS gen_waterfall)

# live_waterfall — raylib-backed real-time spectrogram viewer. Tails
# the .iq file simple_sat_ops is writing and renders a scrolling
# viridis waterfall in a tall narrow window beside the operator UI.
# Optional: skipped if raylib isn't installed.
if (RAYLIB_FOUND)
    add_executable(live_waterfall utils/live_waterfall.c)
    target_include_directories(live_waterfall PRIVATE ${RAYLIB_INCLUDE_DIRS})
    target_link_directories(live_waterfall PRIVATE ${RAYLIB_LIBRARY_DIRS})
    target_link_libraries(live_waterfall PRIVATE ${RAYLIB_LIBRARIES} m)
    # Homebrew's raylib.pc on macOS doesn't list the Cocoa / OpenGL /
    # IOKit frameworks the static lib pulls in (and the dylib variant
    # isn't always installed). Add them explicitly on Darwin; harmless
    # on Linux where these frameworks don't exist (the find_library
    # below returns NOTFOUND-suffixed strings which we filter out).
    if (APPLE)
        target_link_libraries(live_waterfall PRIVATE
            "-framework Cocoa"
            "-framework OpenGL"
            "-framework IOKit"
            "-framework CoreVideo"
            "-framework Foundation")
    endif()
    list(APPEND SSO_TARGETS live_waterfall)

    # decode_inspector — raylib-backed interactive viewer for offline .iq
    # captures. Lets the operator mouse-draw time × freq boxes around
    # bursts they want to feed back into rx_replay --anchor-csv.
    set(DECODE_INSPECTOR_SRCS utils/decode_inspector.c utils/pdf_writer.c
                          src/dsp/modem.c src/dsp/modem_fsk.c
                          src/dsp/modem_viterbi.c src/dsp/sw_nco.c
                          src/proto/golay24.c src/proto/rs.c
                          src/proto/csp.c)
    if (APPLE)
        # raylib/GLFW drop NSEventTypeMagnify (trackpad pinch); this
        # ObjC shim installs a global NSEvent monitor and accumulates
        # magnification into a global the C side reads each frame.
        list(APPEND DECODE_INSPECTOR_SRCS utils/decode_inspector_macos.m)
    endif()
    add_executable(decode_inspector ${DECODE_INSPECTOR_SRCS})
    target_include_directories(decode_inspector PRIVATE ${RAYLIB_INCLUDE_DIRS})
    target_link_directories(decode_inspector PRIVATE ${RAYLIB_LIBRARY_DIRS})
    target_link_libraries(decode_inspector PRIVATE ${RAYLIB_LIBRARIES} waterfall_core m)
    if (APPLE)
        target_link_libraries(decode_inspector PRIVATE
            "-framework Cocoa"
            "-framework AppKit"
            "-framework OpenGL"
            "-framework IOKit"
            "-framework CoreVideo"
            "-framework Foundation")
    endif()
    list(APPEND SSO_TARGETS decode_inspector)
else()
    message(STATUS "raylib not found via pkg-config; live_waterfall + decode_inspector skipped.")
endif()

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

# FSK demod self-test: pins the modem_fsk refactor (per-stage helpers +
# the new modem_fsk_iq_to_bits_diag entry point) on a synthetic MSK
# frame. Same .c links as modem_iq_selftest since modem_fsk depends on
# modem.h.
add_executable(modem_fsk_selftest unit_tests/modem_fsk_selftest.c
               src/dsp/modem.c src/dsp/modem_fsk.c)
target_include_directories(modem_fsk_selftest PRIVATE ${UNIT_TESTS_INCLUDE})
target_link_libraries(modem_fsk_selftest PRIVATE m)
list(APPEND SSO_TARGETS modem_fsk_selftest)

# Software-Doppler NCO selftest. Runs without UHD — the NCO is the
# DSP core, extracted so it can be exercised on synthesised IQ.
add_executable(sw_nco_selftest unit_tests/sw_nco_selftest.c
               src/dsp/sw_nco.c)
target_include_directories(sw_nco_selftest PRIVATE ${UNIT_TESTS_INCLUDE})
target_link_libraries(sw_nco_selftest PRIVATE m)
list(APPEND SSO_TARGETS sw_nco_selftest)

# CTS1 beacon parser selftest. Audits every byte offset, enum mapping,
# and length-anchored sniff in beacon_cts1.c. The flight firmware is
# pinned at sat-1-rc3; a silent regression here shows up as wrong
# fields on the operator panel during a pass.
add_executable(beacon_cts1_selftest unit_tests/beacon_cts1_selftest.c
               src/beacon/beacon_cts1.c)
target_include_directories(beacon_cts1_selftest PRIVATE ${UNIT_TESTS_INCLUDE})
target_link_libraries(beacon_cts1_selftest PRIVATE m)
list(APPEND SSO_TARGETS beacon_cts1_selftest)

# CSP v1 header codec + CRC32 selftest. Pins the 32-bit wire layout
# byte-for-byte, exhaustively round-trips {min,max}^6 corner combos,
# and audits csp_crc32_zlib against IEEE 802.3 reference vectors.
add_executable(csp_selftest unit_tests/csp_selftest.c src/proto/csp.c)
target_include_directories(csp_selftest PRIVATE ${UNIT_TESTS_INCLUDE})
list(APPEND SSO_TARGETS csp_selftest)

# Bandpass biquad selftest. Pins the RBJ-cookbook coefficients, peak
# gain at f0, DC + Nyquist nulls, stopband attenuation, impulse-decay
# stability, Q < 0.5 floor, and cascade-deepens-stopband behaviour.
add_executable(biquad_selftest unit_tests/biquad_selftest.c src/dsp/biquad.c)
target_include_directories(biquad_selftest PRIVATE ${UNIT_TESTS_INCLUDE})
target_link_libraries(biquad_selftest PRIVATE m)
list(APPEND SSO_TARGETS biquad_selftest)

# FIR decimator selftest. Audits constructor validation, accessors,
# DC unity gain, passband transparency, anti-alias rejection (> 30 dB
# in the would-alias stopband), decimation-ratio output count, and
# state continuity across split push() calls.
add_executable(fir_decim_selftest unit_tests/fir_decim_selftest.c
               src/dsp/fir_decim.c)
target_include_directories(fir_decim_selftest PRIVATE ${UNIT_TESTS_INCLUDE})
target_link_libraries(fir_decim_selftest PRIVATE m)
list(APPEND SSO_TARGETS fir_decim_selftest)

# Carrier-presence squelch selftest. Pins init defaults, the state-
# machine transitions (OFF / AUTO_BOOTSTRAPPING → AUTO_ENGAGED / FIXED),
# pass-through behaviour, hold timer, status-string format, and in-/
# out- buffer aliasing.
add_executable(monitor_squelch_selftest
               unit_tests/monitor_squelch_selftest.c
               src/dsp/monitor_squelch.c src/dsp/biquad.c)
target_include_directories(monitor_squelch_selftest PRIVATE ${UNIT_TESTS_INCLUDE})
target_link_libraries(monitor_squelch_selftest PRIVATE m)
list(APPEND SSO_TARGETS monitor_squelch_selftest)

# Broadband-burst detector selftest. Synthesises CW vs wideband noise
# inputs through iq_burst and checks bright_bins lands in the right
# bands (narrow signals → few bins, wideband → many).
add_executable(iq_burst_selftest unit_tests/iq_burst_selftest.c
               src/dsp/iq_burst.c)
target_include_directories(iq_burst_selftest PRIVATE ${UNIT_TESTS_INCLUDE})
target_link_libraries(iq_burst_selftest PRIVATE m)
list(APPEND SSO_TARGETS iq_burst_selftest)

# packet_db selftest. Exercises schema creation, V4 migrations, insert
# validation, dedup tuple, NaN-to-NULL mapping, register_tle
# idempotency, and run_id format. Needs OpenSSL (sha1 of payloads) +
# sqlite3 (the storage backend itself); test also links sqlite3
# directly to audit row content via independent connections.
if (OPENSSL_FOUND AND SQLITE3_FOUND)
    # target_link_packet_db handles sqlite3 + the WITH_SQLITE3 define
    # and source. The selftest also calls sqlite3_open_v2 directly via
    # the public headers to audit row content on a separate connection,
    # which is why we still include SQLITE3_INCLUDE_DIRS explicitly.
    add_executable(packet_db_selftest unit_tests/packet_db_selftest.c)
    target_include_directories(packet_db_selftest PRIVATE
        ${UNIT_TESTS_INCLUDE}
        ${OPENSSL_INCLUDE_DIRS} ${SQLITE3_INCLUDE_DIRS})
    target_link_directories(packet_db_selftest PRIVATE
        ${OPENSSL_LIBRARY_DIRS})
    target_link_libraries(packet_db_selftest PRIVATE
        ${OPENSSL_LIBRARIES} m)
    target_link_packet_db(packet_db_selftest)
    target_link_sso_core(packet_db_selftest)
    list(APPEND SSO_TARGETS packet_db_selftest)
endif()

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

    # Backfill the sent_tcmd table from historical tx.log files.
    add_executable(tcmd_import utils/tcmd_import.c src/proto/agenda_line.c)
    target_include_directories(tcmd_import PRIVATE ${OPENSSL_INCLUDE_DIRS})
    target_link_directories(tcmd_import PRIVATE ${OPENSSL_LIBRARY_DIRS})
    target_link_libraries(tcmd_import PRIVATE ${OPENSSL_LIBRARIES} m)
    target_link_packet_db(tcmd_import)
    target_link_sso_core(tcmd_import)
    list(APPEND SSO_TARGETS tcmd_import)

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

        # Command explorer over the sent_tcmd table (mirror of
        # packet_browser). Links beacon_cts1 for tcmd_response decoding.
        add_executable(tcmd_browser utils/tcmd_browser.c
                       src/beacon/beacon_cts1.c)
        target_compile_definitions(tcmd_browser PRIVATE
            TCMD_BROWSER_HAVE_NCURSES)
        target_include_directories(tcmd_browser PRIVATE
            ${OPENSSL_INCLUDE_DIRS} ${NCURSES_INCLUDE_DIRS})
        target_link_directories(tcmd_browser PRIVATE
            ${OPENSSL_LIBRARY_DIRS} ${NCURSES_LIBRARY_DIRS})
        target_link_libraries(tcmd_browser PRIVATE
            ${OPENSSL_LIBRARIES} ${NCURSES_LIBRARIES} m)
        target_link_packet_db(tcmd_browser)
        target_link_sso_core(tcmd_browser)
        list(APPEND SSO_TARGETS tcmd_browser)
    endif()

    # Offline sliding-window decoder for WAV / raw S16_LE files.
    add_executable(rx_replay utils/rx_replay.c utils/wav_read.c
                   src/pipeline/decode_loop.c
                   src/dsp/modem.c src/dsp/modem_fsk.c
                   src/dsp/modem_iq.c src/dsp/modem_viterbi.c
                   src/dsp/sw_nco.c src/dsp/iq_burst.c
                   src/proto/ax100.c src/proto/rs.c src/proto/golay24.c
                   src/proto/csp.c src/proto/hmac_keyfile.c
                   src/beacon/beacon_cts1.c src/pipeline/rx_tui.c
                   src/orbit/tle_csv.c)
    target_include_directories(rx_replay PRIVATE ${OPENSSL_INCLUDE_DIRS})
    target_link_directories(rx_replay PRIVATE ${OPENSSL_LIBRARY_DIRS})
    target_link_libraries(rx_replay PRIVATE ${OPENSSL_LIBRARIES} m)
    if (SNDFILE_FOUND)
        target_compile_definitions(rx_replay PRIVATE HAVE_SNDFILE)
        target_include_directories(rx_replay PRIVATE ${SNDFILE_INCLUDE_DIRS})
        target_link_directories(rx_replay PRIVATE ${SNDFILE_LIBRARY_DIRS})
        target_link_libraries(rx_replay PRIVATE ${SNDFILE_LIBRARIES})
    endif()
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

    # tx_burst selftest: pins the two simple_sat_ops TX policies the
    # operator depends on every pass — HMAC default-on (key passes
    # through to ax100 trailer; can be disabled by passing key=NULL)
    # and TX Doppler default-on (formula matches f_tx = carrier/(1-rr/c);
    # enable=0 short-circuits to nominal). Stubs out the UHD-touching
    # b210_rx_tx_core_burst symbol so we can link tx_burst.c without
    # libuhd.
    add_executable(tx_burst_selftest unit_tests/tx_burst_selftest.c
                   src/pipeline/tx_burst.c
                   src/proto/csp.c src/proto/ax100.c
                   src/proto/rs.c src/proto/golay24.c
                   src/proto/hmac_keyfile.c
                   src/dsp/modem.c)
    target_include_directories(tx_burst_selftest PRIVATE
        ${UNIT_TESTS_INCLUDE} ${OPENSSL_INCLUDE_DIRS})
    target_link_directories(tx_burst_selftest PRIVATE
        ${OPENSSL_LIBRARY_DIRS})
    target_link_libraries(tx_burst_selftest PRIVATE
        ${OPENSSL_LIBRARIES} m)
    list(APPEND SSO_TARGETS tx_burst_selftest)

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
                       src/dsp/modem.c
                       src/dsp/sw_nco.c
                       src/hw/carrier_trim.c)
        target_include_directories(b210_rx_capture PRIVATE
            ${UHD_INCLUDE_DIRS})
        target_link_directories(b210_rx_capture PRIVATE
            ${UHD_LIBRARY_DIRS})
        target_link_libraries(b210_rx_capture PRIVATE
            ${UHD_LIBRARIES} m)
        target_link_sso_core(b210_rx_capture)
        list(APPEND SSO_TARGETS b210_rx_capture)

        # B210 RX-gain sweep: open at each gain step, dwell, measure
        # mean(I² + Q²). Uses the same b210_rx_tx_core wrapper that
        # simple_sat_ops' live receiver does, so the measurement
        # conditions (rate, decim, FIR cutoff) match operationally.
        add_executable(b210_gain_sweep utils/b210_gain_sweep.c
                       utils/pdf_writer.c
                       src/hw/b210_rx_tx_core.c
                       src/hw/sdr_backend.c src/hw/sdr_uhd.c
                       src/hw/sdr_usb_detect.c
                       src/hw/carrier_trim.c
                       src/dsp/fir_decim.c src/dsp/sw_nco.c
                       src/dsp/iq_burst.c
                       src/dsp/modem.c)
        # WITH_USRP_B210 must be defined for this target too: sdr_backend.c's
        # ops_for() only returns the UHD ops under this macro.
        target_compile_definitions(b210_gain_sweep PRIVATE WITH_USRP_B210)
        target_include_directories(b210_gain_sweep PRIVATE
            ${UHD_INCLUDE_DIRS})
        target_link_directories(b210_gain_sweep PRIVATE
            ${UHD_LIBRARY_DIRS})
        target_link_libraries(b210_gain_sweep PRIVATE
            ${UHD_LIBRARIES} m)
        if (LIBUSB_FOUND)
            target_compile_definitions(b210_gain_sweep PRIVATE HAVE_LIBUSB)
            target_include_directories(b210_gain_sweep PRIVATE ${LIBUSB_INCLUDE_DIRS})
            target_link_directories(b210_gain_sweep PRIVATE ${LIBUSB_LIBRARY_DIRS})
            target_link_libraries(b210_gain_sweep PRIVATE ${LIBUSB_LIBRARIES})
        endif()
        target_link_sso_core(b210_gain_sweep)
        list(APPEND SSO_TARGETS b210_gain_sweep)

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

# ----- SDR pre-flight probe (no OpenSSL / ncurses needed) --------------------
#
# `sdr_probe` reports which UHD device and antenna ports simple_sat_ops
# would use, plus any RTL-SDR dongles. Buildable on any host with an SDR
# library so the operator can confirm the hardware before a pass.
if (WITH_USRP_B210)
    add_executable(sdr_probe utils/sdr_probe.c src/hw/sdr_usb_detect.c)
    target_compile_definitions(sdr_probe PRIVATE WITH_USRP_B210)
    target_include_directories(sdr_probe PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hw ${UHD_INCLUDE_DIRS})
    target_link_directories(sdr_probe PRIVATE ${UHD_LIBRARY_DIRS})
    target_link_libraries(sdr_probe PRIVATE ${UHD_LIBRARIES})
    if (LIBUSB_FOUND)
        target_compile_definitions(sdr_probe PRIVATE HAVE_LIBUSB)
        target_include_directories(sdr_probe PRIVATE ${LIBUSB_INCLUDE_DIRS})
        target_link_directories(sdr_probe PRIVATE ${LIBUSB_LIBRARY_DIRS})
        target_link_libraries(sdr_probe PRIVATE ${LIBUSB_LIBRARIES})
    endif()
    if (WITH_RTL_SDR)
        target_compile_definitions(sdr_probe PRIVATE WITH_RTL_SDR)
        target_include_directories(sdr_probe PRIVATE ${RTLSDR_INCLUDE_DIRS})
        target_link_directories(sdr_probe PRIVATE ${RTLSDR_LIBRARY_DIRS})
        target_link_libraries(sdr_probe PRIVATE ${RTLSDR_LIBRARIES})
    endif()
    list(APPEND SSO_TARGETS sdr_probe)
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

    # Orbital-elements ("keps") summary table. Static element report only,
    # no propagation, so it needs neither ncurses nor prediction.c -- just
    # the library decode helpers plus sso_paths for the dated-TLE default.
    add_executable(tle_keps apps/tle_keps.c src/ipc/sso_paths.c)
    target_link_libraries(tle_keps PRIVATE ${SGP4SDP4_LIB} m)
    list(APPEND SSO_TARGETS tle_keps)

    # TLE from a single GNSS position/velocity state vector. ECEF->inertial,
    # a two-body solve for a guess, then an SGP4 differential correction so
    # the elements reproduce the state under this build's own propagator.
    # Drives SGP4 directly (no select_ephemeris), so just the library + m.
    add_executable(tle_from_state apps/tle_from_state.c src/orbit/bestxyz.c)
    target_link_libraries(tle_from_state PRIVATE ${SGP4SDP4_LIB} m)
    list(APPEND SSO_TARGETS tle_from_state)

    # Live side-by-side TLE comparison (needs ncurses for the live UI).
    if (NCURSES_FOUND)
        add_executable(tle_compare apps/tle_compare.c
                       src/orbit/prediction.c src/orbit/oem.c src/orbit/tle_csv.c)
        target_include_directories(tle_compare PRIVATE ${NCURSES_INCLUDE_DIRS})
        target_link_directories(tle_compare PRIVATE ${NCURSES_LIBRARY_DIRS})
        target_link_libraries(tle_compare PRIVATE
                              ${SGP4SDP4_LIB} ${NCURSES_LIBRARIES} m)
        list(APPEND SSO_TARGETS tle_compare)
    endif()

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
    add_executable(simple_sat_ops apps/main.c apps/cli_args.c
                   src/ui/tui.c src/ui/spectrogram.c src/ui/panels.c
                   src/ui/tx_compose.c src/ui/tx_log.c src/ui/viewer.c
                   src/ui/auto_tcmd.c src/ui/cmd_line.c src/ui/input.c
                   src/control/pass_session.c src/control/scan_sky.c
                   src/control/tracking.c src/control/operator_ipc.c
                   src/control/hw_bringup.c
                   src/orbit/prediction.c src/orbit/oem.c
                   src/orbit/pursuit.c
                   src/hw/antenna_rotator.c
                   src/hw/antenna_rotator_async.c
                   src/hw/rotator_calibrate.c
                   src/hw/tr_switch.c
                   src/beacon/telemetry.c
                   src/pipeline/decode_loop.c
                   src/dsp/modem.c src/dsp/modem_fsk.c
                   src/dsp/modem_iq.c src/dsp/modem_viterbi.c
                   src/proto/ax100.c src/proto/rs.c
                   src/proto/golay24.c src/proto/csp.c
                   src/proto/hmac_keyfile.c
                   src/proto/agenda_line.c
                   src/proto/tcmd_lint.c src/proto/tcmd_spec.c
                   src/proto/sso_pseudo.c
                   src/beacon/beacon_cts1.c
                   src/orbit/tle_csv.c
                   src/dsp/biquad.c src/dsp/monitor_squelch.c)
    target_include_directories(simple_sat_ops PRIVATE
        ${NCURSES_INCLUDE_DIRS} ${OPENSSL_INCLUDE_DIRS})
    target_link_directories(simple_sat_ops PRIVATE
        ${NCURSES_LIBRARY_DIRS} ${OPENSSL_LIBRARY_DIRS})
    target_link_libraries(simple_sat_ops PRIVATE
        ${SGP4SDP4_LIB} ${NCURSES_LIBRARIES} ${OPENSSL_LIBRARIES} m pthread)
    # Shared SDR RX/TX chain + the dispatcher, compiled whenever at least
    # one backend is enabled. SSO_WITH_SDR gates the SDR-general code in
    # apps/main.c (which is now device-agnostic). Per-device backends
    # (sdr_uhd.c / sdr_rtlsdr.c) are added under their own option below.
    if (WITH_USRP_B210 OR WITH_RTL_SDR)
        target_compile_definitions(simple_sat_ops PRIVATE SSO_WITH_SDR)
        target_sources(simple_sat_ops PRIVATE
                       src/hw/b210_rx_tx_core.c
                       src/hw/sdr_backend.c
                       src/hw/carrier_trim.c
                       src/dsp/fir_decim.c src/dsp/sw_nco.c
                       src/dsp/iq_burst.c
                       src/pipeline/rx_session.c src/pipeline/tx_burst.c)
    endif()
    if (WITH_USRP_B210)
        target_compile_definitions(simple_sat_ops PRIVATE WITH_USRP_B210)
        target_sources(simple_sat_ops PRIVATE src/hw/sdr_uhd.c src/hw/sdr_usb_detect.c)
        target_include_directories(simple_sat_ops PRIVATE ${UHD_INCLUDE_DIRS})
        target_link_directories(simple_sat_ops PRIVATE ${UHD_LIBRARY_DIRS})
        target_link_libraries(simple_sat_ops PRIVATE ${UHD_LIBRARIES})
        if (LIBUSB_FOUND)
            target_compile_definitions(simple_sat_ops PRIVATE HAVE_LIBUSB)
            target_include_directories(simple_sat_ops PRIVATE ${LIBUSB_INCLUDE_DIRS})
            target_link_directories(simple_sat_ops PRIVATE ${LIBUSB_LIBRARY_DIRS})
            target_link_libraries(simple_sat_ops PRIVATE ${LIBUSB_LIBRARIES})
        endif()
    endif()
    if (WITH_RTL_SDR)
        target_compile_definitions(simple_sat_ops PRIVATE WITH_RTL_SDR)
        target_sources(simple_sat_ops PRIVATE src/hw/sdr_rtlsdr.c)
        target_include_directories(simple_sat_ops PRIVATE ${RTLSDR_INCLUDE_DIRS})
        target_link_directories(simple_sat_ops PRIVATE ${RTLSDR_LIBRARY_DIRS})
        target_link_libraries(simple_sat_ops PRIVATE ${RTLSDR_LIBRARIES})
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

# simple_sat_ops and the other uplink tools read the HMAC keyfile to sign
# (and, with --hmac, verify) AX100 frames. Operators must be able to run
# them without being able to read the key bytes themselves. The mechanism:
# install each keyfile-reading binary setgid to SSO_RUNNER_GROUP — the
# group allowed to read the key (the key file is
# root:${SSO_RUNNER_GROUP} 0640) — while operators are NOT members of that
# group. The binary's effective group lets it read the key; a setgid
# process is also non-dumpable, so the key can't be scraped out of its
# memory. See the HMAC keyfile notes in the manual.
#
# This has to live in `install` rather than as a one-off chmod on the
# ground machine, because every `sudo make install` copies a fresh binary
# back to plain 0755 and would silently drop the setgid bit.
#
# SSO_HMAC_TARGETS is exactly the set of binaries that #include
# hmac_keyfile.h — keep it in sync with that set. (Display-only tools that
# merely show the hmac_ok DB column, e.g. packet_browser, are not here:
# they never read the key.)
#
# Self-gating: if the group doesn't exist (dev hosts, or a new ground
# machine before its one-time `groupadd ${SSO_RUNNER_GROUP}`), chgrp fails
# and the binary is left at the default 0755 with a clear notice — never
# an install error. Set SSO_RUNNER_GROUP empty to opt out entirely.
set(SSO_RUNNER_GROUP "sso-runner" CACHE STRING
    "Group allowed to read the HMAC keyfile; the uplink tools install setgid to it so operators can TX/verify without reading the key. Empty = plain install.")
set(SSO_HMAC_TARGETS simple_sat_ops uplink_test rx_decode rx_replay tx_frame_sdr)

if (SSO_RUNNER_GROUP)
    foreach(_sso_t IN LISTS SSO_HMAC_TARGETS)
        if (TARGET ${_sso_t})
            install(CODE "
                set(_sso_bin \"\$ENV{DESTDIR}/usr/local/bin/${_sso_t}\")
                execute_process(
                    COMMAND chgrp \"${SSO_RUNNER_GROUP}\" \"\${_sso_bin}\"
                    RESULT_VARIABLE _sso_chgrp_rc
                    ERROR_QUIET)
                if (_sso_chgrp_rc STREQUAL \"0\")
                    # chgrp clears the setgid bit (the kernel does this even
                    # for root on modern Linux), so set the mode AFTER it.
                    execute_process(COMMAND chmod 2755 \"\${_sso_bin}\")
                    message(STATUS
                        \"${_sso_t}: installed setgid ${SSO_RUNNER_GROUP} — HMAC key readable by the program, not by operators\")
                else()
                    message(STATUS
                        \"${_sso_t}: group '${SSO_RUNNER_GROUP}' not present; left at 0755 — HMAC setgid protection NOT applied (run 'sudo groupadd --system ${SSO_RUNNER_GROUP}' on the ground machine, then re-install)\")
                endif()
            ")
        endif()
    endforeach()
endif()

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

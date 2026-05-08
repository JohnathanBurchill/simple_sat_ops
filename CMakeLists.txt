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
pkg_check_modules(SQLITE3 sqlite3)
find_library(SGP4SDP4_LIB sgp4sdp4)

# packet_db is a shared dependency of every receiver: rx_decode,
# rx_replay, rx_live, b210_rx_live all tap emit_frame and write each
# decoded packet into a project-wide SQLite store. When sqlite3 is
# present, packet_db.c compiles against it and WITH_SQLITE3 is defined
# on the targets that use it. When absent, packet_db.c falls back to a
# no-op stub so the binaries still link and run; the receivers' --db
# flags warn and continue.
if (NOT SQLITE3_FOUND)
    message(WARNING "sqlite3 not found via pkg-config; packet_db will "
                    "compile as a no-op stub. Install sqlite3 (`brew "
                    "install sqlite` on macOS) for full DB support.")
endif()
function(target_link_packet_db target)
    target_sources(${target} PRIVATE packet_db.c)
    if (SQLITE3_FOUND)
        target_compile_definitions(${target} PRIVATE WITH_SQLITE3)
        target_include_directories(${target} PRIVATE ${SQLITE3_INCLUDE_DIRS})
        target_link_directories(${target} PRIVATE ${SQLITE3_LIBRARY_DIRS})
        target_link_libraries(${target} PRIVATE ${SQLITE3_LIBRARIES})
    endif()
endfunction()

# USRP B210 backend (requires UHD). Default ON: the FrontierSat ground
# station runs the B210 receiver, so the SDR-side tools (b210_rx_capture,
# b210_rx_live, tx_frame_sdr) are part of the standard build. Disable
# with `-DWITH_USRP_B210=OFF` on hosts without UHD installed; in that
# case radio_usrp_b210.c is still compiled but its init() returns
# RADIO_NOT_IMPLEMENTED so the binaries link without UHD.
option(WITH_USRP_B210 "Build USRP B210 backend (requires UHD)" ON)

# Sources shared by every binary that links the radio dispatcher.
set(RADIO_SOURCES radio.c radio_icom_civ.c radio_yaesu_cat.c radio_usrp_b210.c)
if (WITH_USRP_B210)
    pkg_check_modules(UHD uhd)
    if (NOT UHD_FOUND)
        message(FATAL_ERROR "WITH_USRP_B210=ON but UHD not found via pkg-config. "
                            "Install UHD (e.g. `brew install uhd` on macOS) and retry.")
    endif()
    add_compile_definitions(WITH_USRP_B210)
endif()

# Apply UHD include / link wiring to a target that links the radio
# dispatcher. No-op when WITH_USRP_B210 is OFF — the .c file's #ifdef
# leaves init() as the friendly RADIO_NOT_IMPLEMENTED stub. Mirrors the
# OpenSSL / ALSA per-target wiring style used elsewhere in this file.
function(target_link_radio_uhd target)
    if (WITH_USRP_B210)
        target_include_directories(${target} PRIVATE ${UHD_INCLUDE_DIRS})
        target_link_directories(${target} PRIVATE ${UHD_LIBRARY_DIRS})
        target_link_libraries(${target} PRIVATE ${UHD_LIBRARIES})
    endif()
endfunction()

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
target_link_radio_uhd(radio_ctl)
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

# Beacon-cadence detector for post-pass triage of FM-demoded WAVs.
# Reports times where the (signal-band)/(noise-band) energy ratio
# spikes — an FM-capture-effect signature that lines up with beacon
# transmissions even when the chain is barely above threshold.
# --gate-wav reuses the same logic monitor_squelch runs live.
add_executable(beacon_detect utils/beacon_detect.c utils/wav_read.c
               biquad.c monitor_squelch.c)
target_include_directories(beacon_detect PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(beacon_detect PRIVATE m)
list(APPEND SSO_TARGETS beacon_detect)

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
                   modem.c ax100.c rs.c golay24.c csp.c hmac_keyfile.c
                   beacon_cts1.c decode_loop.c)
    target_include_directories(rx_decode PRIVATE ${CMAKE_SOURCE_DIR} ${OPENSSL_INCLUDE_DIRS})
    target_link_directories(rx_decode PRIVATE ${OPENSSL_LIBRARY_DIRS})
    target_link_libraries(rx_decode PRIVATE ${OPENSSL_LIBRARIES} m)
    target_link_packet_db(rx_decode)
    list(APPEND SSO_TARGETS rx_decode)

    # Non-interactive query CLI over the packet DB. Read-only; safe to
    # run while a receiver is filling the DB in another shell. No
    # OpenSSL or ncurses needed — just SQLite. When sqlite3 isn't
    # available the binary is still built but its main() prints a
    # helpful "rebuild with libsqlite3-dev" message and exits.
    add_executable(packet_query utils/packet_query.c)
    target_include_directories(packet_query PRIVATE
        ${CMAKE_SOURCE_DIR} ${OPENSSL_INCLUDE_DIRS})
    target_link_directories(packet_query PRIVATE ${OPENSSL_LIBRARY_DIRS})
    target_link_libraries(packet_query PRIVATE ${OPENSSL_LIBRARIES} m)
    target_link_packet_db(packet_query)
    list(APPEND SSO_TARGETS packet_query)

    # Curses TUI sibling of packet_query. Same DB, same filters, but
    # with a scrolling list and a detail panel that polls at ~1 Hz so
    # live decodes appear while a receiver is running. ncurses is
    # required; without it the binary is still built but main() prints
    # a friendly stderr message.
    if (NCURSES_FOUND)
        add_executable(packet_browser utils/packet_browser.c)
        target_compile_definitions(packet_browser PRIVATE
            PACKET_BROWSER_HAVE_NCURSES)
        target_include_directories(packet_browser PRIVATE
            ${CMAKE_SOURCE_DIR} ${OPENSSL_INCLUDE_DIRS}
            ${NCURSES_INCLUDE_DIRS})
        target_link_directories(packet_browser PRIVATE
            ${OPENSSL_LIBRARY_DIRS} ${NCURSES_LIBRARY_DIRS})
        target_link_libraries(packet_browser PRIVATE
            ${OPENSSL_LIBRARIES} ${NCURSES_LIBRARIES} m)
        target_link_packet_db(packet_browser)
        list(APPEND SSO_TARGETS packet_browser)
    endif()

    # Offline equivalent of rx_live: same sliding-window decode loop,
    # same partial-RS rescue, same position-quantised dedup, fed from
    # a WAV / raw S16_LE file rather than ALSA. Mac-buildable.
    add_executable(rx_replay utils/rx_replay.c utils/wav_read.c decode_loop.c
                   modem.c ax100.c rs.c golay24.c csp.c hmac_keyfile.c
                   beacon_cts1.c rx_tui.c)
    target_include_directories(rx_replay PRIVATE ${CMAKE_SOURCE_DIR} ${OPENSSL_INCLUDE_DIRS})
    target_link_directories(rx_replay PRIVATE ${OPENSSL_LIBRARY_DIRS})
    target_link_libraries(rx_replay PRIVATE ${OPENSSL_LIBRARIES} m)
    if (NCURSES_FOUND)
        target_compile_definitions(rx_replay PRIVATE RX_TUI_AVAILABLE)
        target_include_directories(rx_replay PRIVATE ${NCURSES_INCLUDE_DIRS})
        target_link_directories(rx_replay PRIVATE ${NCURSES_LIBRARY_DIRS})
        target_link_libraries(rx_replay PRIVATE ${NCURSES_LIBRARIES})
    endif()
    target_link_packet_db(rx_replay)
    list(APPEND SSO_TARGETS rx_replay)

    # CTS1 beacon WAV generator: fills the firmware-canonical packed
    # struct with deterministic plausibles, frames CSP+AX100, writes a
    # 48 kHz mono WAV that round-trips through rx_decode/rx_replay/rx_live.
    add_executable(beacon_gen utils/beacon_gen.c
                   csp.c golay24.c ax100.c rs.c hmac_keyfile.c modem.c)
    target_include_directories(beacon_gen PRIVATE ${CMAKE_SOURCE_DIR} ${OPENSSL_INCLUDE_DIRS})
    target_link_directories(beacon_gen PRIVATE ${OPENSSL_LIBRARY_DIRS})
    target_link_libraries(beacon_gen PRIVATE ${OPENSSL_LIBRARIES} m)
    list(APPEND SSO_TARGETS beacon_gen)

    # SDR-side single-shot frame transmitter: CSP+AX100 -> modem PCM ->
    # software FM modulate -> IQ -> UHD TX streamer. Mac-buildable, no
    # ALSA / no CAT — replaces tx_frame's audio + radio half with
    # libuhd. Only built when both OPENSSL and UHD are available.
    if (WITH_USRP_B210)
        add_executable(tx_frame_sdr utils/tx_frame_sdr.c
                       csp.c golay24.c ax100.c rs.c hmac_keyfile.c modem.c)
        target_include_directories(tx_frame_sdr PRIVATE
            ${CMAKE_SOURCE_DIR} ${OPENSSL_INCLUDE_DIRS} ${UHD_INCLUDE_DIRS})
        target_link_directories(tx_frame_sdr PRIVATE
            ${OPENSSL_LIBRARY_DIRS} ${UHD_LIBRARY_DIRS})
        target_link_libraries(tx_frame_sdr PRIVATE
            ${OPENSSL_LIBRARIES} ${UHD_LIBRARIES} m)
        list(APPEND SSO_TARGETS tx_frame_sdr)

        # SDR-side RX capture: tune the B210, stream IQ for a duration,
        # write a mono FM-demoded WAV (default) or real-part WAV plus,
        # optionally, the raw int16 IQ stream. ALSA is optional: when
        # present the binary picks up --monitor support so the demoded
        # audio plays on a system speaker in real time alongside the
        # WAV write. Mac-buildable (no ALSA there → --monitor reports
        # "this binary was built without ALSA").
        add_executable(b210_rx_capture utils/b210_rx_capture.c modem.c)
        target_include_directories(b210_rx_capture PRIVATE
            ${CMAKE_SOURCE_DIR} ${UHD_INCLUDE_DIRS})
        target_link_directories(b210_rx_capture PRIVATE
            ${UHD_LIBRARY_DIRS})
        target_link_libraries(b210_rx_capture PRIVATE
            ${UHD_LIBRARIES} m)
        if (ASOUND_FOUND)
            target_compile_definitions(b210_rx_capture PRIVATE WITH_ALSA)
            target_include_directories(b210_rx_capture PRIVATE ${ASOUND_INCLUDE_DIRS})
            target_link_directories(b210_rx_capture PRIVATE ${ASOUND_LIBRARY_DIRS})
            target_link_libraries(b210_rx_capture PRIVATE ${ASOUND_LIBRARIES})
        endif()
        list(APPEND SSO_TARGETS b210_rx_capture)

        # Live B210 RX + AX100 decoder + Doppler retune driver. Single
        # process owner of the B210; sibling of rx_live (FT-991A audio
        # path) and b210_rx_capture (record-only). Doppler tracking is
        # gated on libsgp4sdp4 — without it, the binary still builds but
        # only supports --no-doppler / fixed-freq mode. ncurses is
        # optional and enables --ui.
        add_executable(b210_rx_live utils/b210_rx_live.c b210_rx_core.c
                       fir_decim.c biquad.c monitor_squelch.c
                       decode_loop.c modem.c ax100.c rs.c golay24.c csp.c
                       hmac_keyfile.c beacon_cts1.c rx_tui.c)
        target_include_directories(b210_rx_live PRIVATE
            ${CMAKE_SOURCE_DIR} ${OPENSSL_INCLUDE_DIRS} ${UHD_INCLUDE_DIRS})
        target_link_directories(b210_rx_live PRIVATE
            ${OPENSSL_LIBRARY_DIRS} ${UHD_LIBRARY_DIRS})
        target_link_libraries(b210_rx_live PRIVATE
            ${OPENSSL_LIBRARIES} ${UHD_LIBRARIES} m pthread)
        if (SGP4SDP4_LIB)
            target_compile_definitions(b210_rx_live PRIVATE WITH_SGP4SDP4)
            target_sources(b210_rx_live PRIVATE prediction.c oem.c)
            target_link_libraries(b210_rx_live PRIVATE ${SGP4SDP4_LIB})
        endif()
        if (NCURSES_FOUND)
            target_compile_definitions(b210_rx_live PRIVATE RX_TUI_AVAILABLE)
            target_include_directories(b210_rx_live PRIVATE ${NCURSES_INCLUDE_DIRS})
            target_link_directories(b210_rx_live PRIVATE ${NCURSES_LIBRARY_DIRS})
            target_link_libraries(b210_rx_live PRIVATE ${NCURSES_LIBRARIES})
        endif()
        if (ASOUND_FOUND)
            target_compile_definitions(b210_rx_live PRIVATE WITH_ALSA)
            target_include_directories(b210_rx_live PRIVATE ${ASOUND_INCLUDE_DIRS})
            target_link_directories(b210_rx_live PRIVATE ${ASOUND_LIBRARY_DIRS})
            target_link_libraries(b210_rx_live PRIVATE ${ASOUND_LIBRARIES})
        endif()
        target_link_packet_db(b210_rx_live)
        list(APPEND SSO_TARGETS b210_rx_live)
    endif()
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
    target_link_radio_uhd(tx_tone)
    list(APPEND SSO_TARGETS tx_tone)

    # Full-band white noise — TX/RX response characterisation
    add_executable(tx_white_noise tx_white_noise.c ${RADIO_SOURCES} radio_device_store.c audio.c)
    target_link_directories(tx_white_noise PRIVATE ${ASOUND_LIBRARY_DIRS})
    target_link_libraries(tx_white_noise PRIVATE m ${ASOUND_LIBRARIES})
    target_link_radio_uhd(tx_white_noise)
    list(APPEND SSO_TARGETS tx_white_noise)

    # Receive-side capture: SignaLink / radio USB CODEC -> WAV + spectrogram.
    # No radio dispatcher dependency — pure audio in.
    add_executable(rx_capture rx_capture.c audio.c)
    target_link_directories(rx_capture PRIVATE ${ASOUND_LIBRARY_DIRS})
    target_link_libraries(rx_capture PRIVATE m ${ASOUND_LIBRARIES})
    list(APPEND SSO_TARGETS rx_capture)

    if (OPENSSL_FOUND)
        # Single-shot frame transmitter: CSP+AX100 -> PCM -> CAT PTT + ALSA playback
        add_executable(tx_frame utils/tx_frame.c csp.c golay24.c ax100.c rs.c hmac_keyfile.c modem.c radio_device_store.c audio.c ${RADIO_SOURCES})
        target_include_directories(tx_frame PRIVATE ${CMAKE_SOURCE_DIR} ${OPENSSL_INCLUDE_DIRS})
        target_link_directories(tx_frame PRIVATE ${OPENSSL_LIBRARY_DIRS} ${ASOUND_LIBRARY_DIRS})
        target_link_libraries(tx_frame PRIVATE ${OPENSSL_LIBRARIES} m ${ASOUND_LIBRARIES})
        target_link_radio_uhd(tx_frame)
        list(APPEND SSO_TARGETS tx_frame)

        # Live AX100 decoder over ALSA capture -> stdout / log file.
        # No radio dispatcher — pure audio in + decode + print.
        add_executable(rx_live rx_live.c decode_loop.c modem.c ax100.c rs.c golay24.c csp.c hmac_keyfile.c audio.c beacon_cts1.c rx_tui.c)
        target_include_directories(rx_live PRIVATE ${CMAKE_SOURCE_DIR} ${OPENSSL_INCLUDE_DIRS})
        target_link_directories(rx_live PRIVATE ${OPENSSL_LIBRARY_DIRS} ${ASOUND_LIBRARY_DIRS})
        target_link_libraries(rx_live PRIVATE ${OPENSSL_LIBRARIES} m ${ASOUND_LIBRARIES})
        if (NCURSES_FOUND)
            target_compile_definitions(rx_live PRIVATE RX_TUI_AVAILABLE)
            target_include_directories(rx_live PRIVATE ${NCURSES_INCLUDE_DIRS})
            target_link_directories(rx_live PRIVATE ${NCURSES_LIBRARY_DIRS})
            target_link_libraries(rx_live PRIVATE ${NCURSES_LIBRARIES})
        endif()
        target_link_packet_db(rx_live)
        list(APPEND SSO_TARGETS rx_live)
    endif()
else()
    message(STATUS "libasound not found; tx_tone, tx_white_noise, tx_frame, rx_capture, rx_live will not be built.")
endif()

# ----- Optional: full main UI (needs ncurses + SGP4; ALSA optional) ---------

# ALSA is not required to run the rotator + radio. When the host has it,
# simple_sat_ops gets the real audio.c (capture + spectrogram), otherwise
# it links audio_stub.c (no-op stubs that complain if reached) so a Mac
# laptop can drive the antenna and Yaesu without libasound.

if (NCURSES_FOUND AND SGP4SDP4_LIB)
    if (ASOUND_FOUND)
        add_executable(simple_sat_ops main.c prediction.c ${RADIO_SOURCES} radio_device_store.c antenna_rotator.c audio.c telemetry.c oem.c)
        target_link_directories(simple_sat_ops PRIVATE ${ASOUND_LIBRARY_DIRS} ${NCURSES_LIBRARY_DIRS})
        target_link_libraries(simple_sat_ops PRIVATE ${SGP4SDP4_LIB} ${NCURSES_LIBRARIES} m ${ASOUND_LIBRARIES} pthread)
    else()
        add_executable(simple_sat_ops main.c prediction.c ${RADIO_SOURCES} radio_device_store.c antenna_rotator.c audio_stub.c telemetry.c oem.c)
        target_link_directories(simple_sat_ops PRIVATE ${NCURSES_LIBRARY_DIRS})
        target_link_libraries(simple_sat_ops PRIVATE ${SGP4SDP4_LIB} ${NCURSES_LIBRARIES} m pthread)
    endif()
    target_link_radio_uhd(simple_sat_ops)
    list(APPEND SSO_TARGETS simple_sat_ops)
else()
    message(STATUS "simple_sat_ops requires ncurses + sgp4sdp4; skipping.")
endif()

# ----- Install --------------------------------------------------------------

install(TARGETS ${SSO_TARGETS} DESTINATION $ENV{HOME}/bin)
install(PROGRAMS scripts/upcoming_amateur_satellite_passes.sh DESTINATION $ENV{HOME}/bin)
install(PROGRAMS scripts/pcm2wav.sh DESTINATION $ENV{HOME}/bin)

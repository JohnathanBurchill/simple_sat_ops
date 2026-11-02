/*

   Simple Satellite Operations  adcs_sdlog.h

   Copyright (C) 2026  Johnathan K Burchill

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef ADCS_SDLOG_H
#define ADCS_SDLOG_H

// Reading the ADCS's SD-card telemetry logs: the ADCS/log_XXXX.TLM files the
// team pulls down with the adcs_get_latest_sd_file_v2 blob, which arrive as
// ordinary bulk_file packets.
//
// A log is a 10-byte mask naming the telemetry frames it records, then one
// record per logging period: the ADCS's own time (uint32 unix seconds, then
// uint16 milliseconds, both little-endian), followed by every selected 6-byte
// frame in the order of its bit in the mask, lowest first. The bits are the
// flight firmware's ADCS_SD_LOG_MASK_* table in adcs_command_ids.h: frame n is
// byte n / 8 of the mask, value 1 << (n % 8). So every record of a log is the
// same length, and record i starts at byte 10 + i * (6 + 6 * frames).
//
// The records are also exactly periodic. Across four logs decoded in full, at
// 10 and 60 seconds, the worst departure of a record from t0 + i * period was
// 0.1 s. That is what ties a downloaded chunk to its log without knowing which
// file the chunk came from -- a bulk_file packet carries an offset, never a
// name: the chunk belongs to the log if every record time inside it falls
// where the log's schedule puts it. A chunk of any other file passes that
// only if its bytes happen to spell out the right 32-bit time, to the second,
// at every record start it holds.

#include <stddef.h>
#include <stdint.h>

#define ADCS_SDLOG_MASK_LEN   10
#define ADCS_SDLOG_STAMP_LEN  6
#define ADCS_SDLOG_FRAME_LEN  6

// The frames read out of a log here, by their bit in the mask. Frame 27 is
// telemetry 132, the same six bytes the extended beacon carries as
// adcs_current_state_1, so beacon_ext_adcs_state() reads it.
#define ADCS_SDLOG_FRAME_STATE       27
// Roll, pitch and yaw of the body frame against the orbit frame, int16
// little-endian, in hundredths of a degree.
#define ADCS_SDLOG_FRAME_EST_ANGLES  28

// How far a record's time may sit from its place on the schedule and still
// count as on it. Ten times the worst departure measured.
#define ADCS_SDLOG_SCHEDULE_TOL_S  1.0

typedef struct {
    uint8_t mask[ADCS_SDLOG_MASK_LEN];
    int     nframes;
    long    rec_len;    // bytes per record, time stamp included
    double  t0;         // the first record's time, unix seconds
} adcs_sdlog_t;

// Read the start of a log: at least the mask and the first record's time
// stamp, 16 bytes. Returns 1 if they read as a log -- a mask selecting at
// least one frame and a time stamp in the mission years -- and 0 otherwise.
int adcs_sdlog_parse_header(const uint8_t *d, size_t n, adcs_sdlog_t *log);

// Where record i starts in the file.
long adcs_sdlog_record_offset(const adcs_sdlog_t *log, long i);

// Where frame `frame` sits within a record, or -1 if the mask does not
// select it.
int adcs_sdlog_frame_offset(const adcs_sdlog_t *log, int frame);

// A record's time, unix seconds, from its 6-byte stamp; -1 if the
// milliseconds read 1000 or more, which no real stamp does.
double adcs_sdlog_stamp(const uint8_t *p);

// The records whose time stamps lie wholly inside the n bytes at file offset
// off: returns how many, with the first and last of them in *first and
// *last. A stamp cut by either end of the chunk is not counted.
long adcs_sdlog_stamps_in(const adcs_sdlog_t *log, long off, size_t n,
                          long *first, long *last);

// The logging period, whole seconds, that would put record i at time t, or 0
// if no period up to the 65535 s the ADCS can be set to does so within
// ADCS_SDLOG_SCHEDULE_TOL_S.
long adcs_sdlog_period_at(const adcs_sdlog_t *log, long i, double t);

// Whether the n bytes at file offset off are a piece of this log, logging
// every `period` seconds: the number of record stamps inside them, every one
// of them on the schedule; 0 if they hold no whole stamp to judge by; -1 if
// any stamp is off it.
long adcs_sdlog_chunk_fits(const adcs_sdlog_t *log, long period, long off,
                           const uint8_t *d, size_t n);

#endif

/*

   Simple Satellite Operations  bulk_size.h

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

#ifndef BULK_SIZE_H
#define BULK_SIZE_H

// How long a downlinked file really is, and which of its chunk offsets can be
// believed. Both facts are needed before a list of missing packets means
// anything, and neither can be read off the bulk_file packets themselves.
//
// A bulk_file packet carries only [file_offset][data] -- never the file's
// length -- so a reassembler that sizes the file as max(offset + length) is
// really reporting "the largest offset I saw", which one bit-flipped offset
// field drags arbitrarily upward. In the 2026-07-21 MPI download a single chunk
// claiming offset 2051784 sized the file at 2051979 bytes instead of its true
// 556728, so a complete file read as 27% recovered and the missing-packet list
// was mostly a demand for bytes that do not exist.
//
// The satellite does report the length, in its log stream:
//
//   [S:TCMD:INFO]: Blob (bulk_downlink_start_blob) args_str: 'mpi_data/2026-07-21.mpi;0;0'
//   [A:LFS:INFO]: Bulk downlink complete. 556728 bytes (2856 packets) downlinked. File: mpi_data/2026-07-21.mpi
//
// The "complete" line counts the bytes THAT DOWNLOAD sent, so it is the file
// length only when the download started at offset 0 and was not cut short by a
// byte cap. Pairing it with the start line that preceded it recovers the length
// in the other cases too: the 2026-08-28 pull of the same file started at
// 307125 and reported 249603 bytes, and 307125 + 249603 = 556728 exactly.

#include <stddef.h>

#define BULK_SIZE_PATH_LEN 128

// No file the satellite holds approaches this, so a length beyond it is a
// misread line rather than a real file.
#define BULK_SIZE_MAX_PLAUSIBLE (2 * 1024 * 1024)

// One download sends at most this much, whatever it was asked for: the firmware
// clamps max_bytes to COMMS_bulk_file_downlink_max_allowable_total_bytes in
// bulk_file_downlink.c, and asking for 0 ("to the end of the file") is clamped
// the same way. So a download that reports exactly this much did not reach the
// end of the file, it ran out of allowance -- mpi_data/2026-08-08.mpi was pulled
// from offset 0 with no cap and reported 1000000 bytes, which is where its
// length stops being known and starts being a floor.
#define BULK_SIZE_FIRMWARE_CAP  1000000L

// A download's start line and the "complete" line that closes it arrive in the
// same pass. Refusing to pair them across a longer gap than this keeps a
// complete whose own start line was never decoded from being paired with a
// stale start offset, which would overstate the file's length. Unpaired, it is
// simply read as a download from offset 0, which understates it instead -- and
// understating is safe, because the length is only ever taken as a maximum.
#define BULK_SIZE_PAIR_WINDOW_MS (2.0 * 60.0 * 60.0 * 1000.0)

typedef struct {
    char path[BULK_SIZE_PATH_LEN];   // file on the satellite
    long size;                       // best known length in bytes
    int  exact;                      // a download that reached EOF reported it
    long pend_start;                 // start offset of the download in flight, -1 if none
    long pend_count;                 // its byte cap, 0 for "to the end of the file"
    double pend_ms;                  // when that start line was received
} bulk_size_entry_t;

typedef struct {
    bulk_size_entry_t *v;
    int n;
    int cap;
} bulk_size_table_t;

// Everything a "Bulk downlink complete" line says.
typedef struct {
    char path[BULK_SIZE_PATH_LEN];
    long bytes;
    long packets;
} bulk_complete_t;

// Everything a bulk-downlink start says, in either the blob-log form above or
// the direct comms_bulk_file_downlink_start(path,start,count) form.
typedef struct {
    char path[BULK_SIZE_PATH_LEN];
    long start;
    long count;   // 0 means "to the end of the file"
} bulk_start_t;

// Pull one record out of a log packet's text. Both return 1 on a match, 0
// otherwise. Log packets are routinely garbled by uncorrectable RS blocks, so
// both insist on the whole literal shape of the line and on plausible numbers,
// and a line that does not parse is simply not a record.
int bulk_parse_complete(const char *text, bulk_complete_t *out);
int bulk_parse_start(const char *text, bulk_start_t *out);

// Feed every log packet's text, oldest first, with its reception time in unix
// ms. Order matters only for pairing a "complete" with its own start line.
void bulk_size_feed(bulk_size_table_t *t, const char *text, double ts_ms);

// The satellite-reported length of `path`, or 0 if nothing in the log said.
// *exact, when given, comes back 1 if a download that ran to the end of the
// file reported the length and 0 if it is only a lower bound.
long bulk_size_lookup(const bulk_size_table_t *t, const char *path, int *exact);

void bulk_size_free(bulk_size_table_t *t);

// Which offsets a download could really have used, learned from the chunks
// whose CRC32 checked out.
//
// The firmware streams a file in fixed 195-byte packets from the offset it was
// asked for, so every real offset of one download is start + k*195: they all
// share a residue mod 195, one residue per commanded start offset. An offset
// field that took a bit flip lands anywhere.
//
// A packet's offset field is covered by the same CSP CRC32 as its data, so a
// packet that passed its CRC has an offset that can be believed outright, and a
// packet that failed has one that cannot. That split is stark in the store: of
// the chunks belonging to the 2026-07-21 MPI experiment, the 3522 CRC-verified
// ones use exactly two residues -- 0 and 45, the two offsets the downloads were
// commanded from -- and not one lands anywhere else, while the 1796 that failed
// their CRC are scattered over 113 residues.
//
// So the grid is learned from verified offsets only, and then used to judge the
// unverified ones, whose data is still worth having but whose position is a
// claim rather than a fact. This matters beyond the file's length: an offset
// that is wrong but lands below the end of the file writes another file's bytes
// into the middle of this one, where they read back as received, so the bytes
// they displaced drop off the missing list instead of being re-requested.
//
// grid[] is 195 flags, one per residue. Fewer than BULK_GRID_MIN_VERIFIED
// verified chunks is not enough to know a download's grid, so the whole grid is
// opened rather than guessed -- an unverified offset is then no worse off than
// it was before.
#define BULK_GRID_MIN_VERIFIED 20

void bulk_grid_learn(const long *off, const unsigned char *verified, int n,
                     unsigned char grid[195]);

// 1 if `off` could be a real offset of a download on this grid.
int bulk_grid_ok(const unsigned char grid[195], long off);

#endif

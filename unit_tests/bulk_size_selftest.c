/*

    Simple Satellite Operations  unit_tests/bulk_size_selftest.c

    Exercises src/proto/bulk_size.c: reading a downlinked file's true length
    out of the satellite's log stream, and telling a real bulk-file chunk
    offset from a bit-flipped one.

    The log lines below are verbatim from the packet store (the 2026-07-21 MPI
    file), including the garbled ones, so the parser is measured against what
    the radio actually delivers rather than against fixtures shaped to suit it.

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

#include "tap.h"
#include "bulk_size.h"

#include <stdlib.h>
#include <string.h>

// Real lines, copied out of packet_db.sqlite.
#define MPI_PATH  "mpi_data/2026-07-21.mpi"

static const char *L_COMPLETE_FULL =
    "\x03""1784756948000+0000715619_E [A:LFS:INFO]: Bulk downlink complete. "
    "556728 bytes (2856 packets) downlinked. File: mpi_data/2026-07-21.mpi ";

static const char *L_COMPLETE_PARTIAL =
    "\x03""1787936479000+0023925728_E [A:LFS:INFO]: Bulk downlink complete. "
    "249603 bytes (1281 packets) downlinked. File: mpi_data/2026-07-21.mpi ";

static const char *L_COMPLETE_CAPPED =
    "\x03""1787983333000+0005416744_E [A:LFS:INFO]: Bulk downlink complete. "
    "371085 bytes (1903 packets) downlinked. File: mpi_data/2026-07-21.mpi ";

static const char *L_START_ZERO =
    "\x03""1784762950000+0000290362_E [S:TCMD:INFO]: Blob "
    "(bulk_downlink_start_blob) args_str: 'mpi_data/2026-07-21.mpi;0;0' ";

static const char *L_START_307125 =
    "\x03""1787936479000+0023610228_E [S:TCMD:INFO]: Blob "
    "(bulk_downlink_start_blob) args_str: 'mpi_data/2026-07-21.mpi;307125;0' ";

static const char *L_START_CAPPED =
    "\x03""1787983333000+0005507280_E [S:TCMD:INFO]: Blob "
    "(bulk_downlink_start_blob) args_str: 'mpi_data/2026-07-21.mpi;0;371085' ";

// An uncorrectable RS block left this one in the store looking like this.
static const char *L_START_GARBLED =
    "\x03""17855q7#67080;0023847328_E [S:TCMD:IJFO]: Blob "
    "(bulk_downlink_stArt_blob) args^_str: 'ADCSolo_bf7rnTLM;0;0 \x1a";

static const char *L_UNRELATED =
    "\x03""1784762950000+0000294508_E [S:LFS:INFO]: Successfully opened file: "
    "mpi_data/2026-07-21.mpi ";

// ---- line parsing ------------------------------------------------------

static void test_parse_complete(void)
{
    bulk_complete_t c = {0};
    tap_ok(bulk_parse_complete(L_COMPLETE_FULL, &c) == 1, "a complete line parses");
    tap_okf(c.bytes == 556728, "byte count read (got %ld, want 556728)", c.bytes);
    tap_okf(c.packets == 2856, "packet count read (got %ld, want 2856)", c.packets);
    tap_okf(strcmp(c.path, MPI_PATH) == 0, "path read (got '%s')", c.path);
}

static void test_parse_complete_rejects(void)
{
    bulk_complete_t c = {0};
    tap_ok(bulk_parse_complete(L_UNRELATED, &c) == 0, "an unrelated log line is not a complete");
    tap_ok(bulk_parse_complete(L_START_ZERO, &c) == 0, "a start line is not a complete");
    tap_ok(bulk_parse_complete(NULL, &c) == 0, "NULL text is not a complete");
    tap_ok(bulk_parse_complete("Bulk downlink complete. ", &c) == 0,
           "the marker with nothing after it is not a complete");
    tap_ok(bulk_parse_complete("Bulk downlink complete. 556728 bytes (2856 packets) "
                               "downlinked. File: ", &c) == 0,
           "a complete with an empty path is rejected");
    // A flipped digit run that overflows a plausible length must not be taken
    // as a real answer -- this is the exact failure the file-size fix exists
    // to prevent, arriving through the log instead of through an offset.
    tap_ok(bulk_parse_complete("Bulk downlink complete. 99999999 bytes (2856 packets) "
                               "downlinked. File: x", &c) == 0,
           "an implausibly large length is rejected");
    // The separator text itself must match, or a corrupted line could be read
    // as a different, valid-looking record.
    tap_ok(bulk_parse_complete("Bulk downlink complete. 556728 bytes [2856 packets) "
                               "downlinked. File: x", &c) == 0,
           "a corrupted separator is rejected");
    tap_ok(bulk_parse_complete("Bulk downlink complete. 556728 bytes (2856 packets) "
                               "downlinked. Path: x", &c) == 0,
           "a corrupted tail is rejected");
}

static void test_parse_start(void)
{
    bulk_start_t s = {0};
    tap_ok(bulk_parse_start(L_START_307125, &s) == 1, "a start line parses");
    tap_okf(s.start == 307125, "start offset read (got %ld, want 307125)", s.start);
    tap_okf(s.count == 0, "uncapped count read as 0 (got %ld)", s.count);
    tap_okf(strcmp(s.path, MPI_PATH) == 0, "path read (got '%s')", s.path);

    memset(&s, 0, sizeof s);
    tap_ok(bulk_parse_start(L_START_CAPPED, &s) == 1, "a capped start line parses");
    tap_okf(s.start == 0 && s.count == 371085,
            "byte cap read (got start %ld count %ld)", s.start, s.count);

    // The direct firmware command form, comma-separated.
    memset(&s, 0, sizeof s);
    tap_ok(bulk_parse_start("comms_bulk_file_downlink_start(mpi_data/x.mpi,160485,1170)", &s) == 1,
           "the direct command form parses");
    tap_okf(s.start == 160485 && s.count == 1170 && strcmp(s.path, "mpi_data/x.mpi") == 0,
            "direct form fields read (got '%s' %ld %ld)", s.path, s.start, s.count);

    // What the ground actually transmits, verbatim out of sent_tcmd. This is
    // the only record of a download whose own log lines were never decoded.
    memset(&s, 0, sizeof s);
    tap_ok(bulk_parse_start(
               "CTS1+exec_blob_from_fs(blobs/bulk_downlink_start_v2.blob,0,"
               "mpi_data/2026-07-21.mpi;306000;0)@tssent=1784666400000"
               "@tsexec=1784666400000!", &s) == 1,
           "the transmitted command form parses");
    tap_okf(s.start == 306000 && s.count == 0 && strcmp(s.path, MPI_PATH) == 0,
            "transmitted form fields read (got '%s' %ld %ld)", s.path, s.start, s.count);
}

static void test_parse_start_rejects(void)
{
    bulk_start_t s = {0};
    tap_ok(bulk_parse_start(L_START_GARBLED, &s) == 0, "a garbled start line is rejected");
    tap_ok(bulk_parse_start(L_UNRELATED, &s) == 0, "an unrelated log line is not a start");
    tap_ok(bulk_parse_start(NULL, &s) == 0, "NULL text is not a start");
    tap_ok(bulk_parse_start("(bulk_downlink_start_blob) args_str: 'p;12'", &s) == 0,
           "a start missing its third field is rejected");
    tap_ok(bulk_parse_start("(bulk_downlink_start_blob) args_str: ';0;0'", &s) == 0,
           "a start with an empty path is rejected");
    tap_ok(bulk_parse_start("(bulk_downlink_start_blob) args_str: 'p;x;0'", &s) == 0,
           "a start with a non-numeric offset is rejected");
    // exec_blob_from_fs runs any blob; only the bulk-downlink one says anything
    // about a file's length.
    tap_ok(bulk_parse_start("CTS1+exec_blob_from_fs(blobs/deploy_antenna.blob,0,x;0;0)!", &s) == 0,
           "exec_blob_from_fs of another blob is not a download start");
}

// ---- the table ---------------------------------------------------------

// Sequences below use round unix-ms values; only the differences matter.
#define T0 1784756000000.0

static void test_size_from_full_download(void)
{
    bulk_size_table_t t = {0};
    bulk_size_feed(&t, L_START_ZERO, T0);
    bulk_size_feed(&t, L_COMPLETE_FULL, T0 + 60000.0);

    int exact = 0;
    long sz = bulk_size_lookup(&t, MPI_PATH, &exact);
    tap_okf(sz == 556728, "a whole-file download gives the length (got %ld, want 556728)", sz);
    tap_ok(exact == 1, "a whole-file download is an exact length");
    bulk_size_free(&t);
}

static void test_size_from_partial_download(void)
{
    // The 2026-08-28 pull started at 307125 and ran to the end: the length is
    // the sum, which is the whole point of pairing the two lines.
    bulk_size_table_t t = {0};
    bulk_size_feed(&t, L_START_307125, T0);
    bulk_size_feed(&t, L_COMPLETE_PARTIAL, T0 + 60000.0);

    int exact = 0;
    long sz = bulk_size_lookup(&t, MPI_PATH, &exact);
    tap_okf(sz == 556728, "start + bytes gives the length (got %ld, want 556728)", sz);
    tap_ok(exact == 1, "a partial download that reached EOF is exact");
    bulk_size_free(&t);
}

static void test_capped_download_is_only_a_lower_bound(void)
{
    // 0;371085 sent exactly its cap, so the file is at least that long and the
    // download says nothing about where it ends.
    bulk_size_table_t t = {0};
    bulk_size_feed(&t, L_START_CAPPED, T0);
    bulk_size_feed(&t, L_COMPLETE_CAPPED, T0 + 60000.0);

    int exact = 1;
    long sz = bulk_size_lookup(&t, MPI_PATH, &exact);
    tap_okf(sz == 371085, "a capped download gives its cap (got %ld, want 371085)", sz);
    tap_ok(exact == 0, "a download stopped by its cap is not an exact length");
    bulk_size_free(&t);
}

static void test_firmware_cap_is_not_a_length(void)
{
    // mpi_data/2026-08-08.mpi, verbatim: asked for the whole file from offset 0
    // and sent exactly 1000000 bytes, because that is all the firmware will
    // send in one download. Reading that as the file's length would report a
    // file of unknown size as complete and ask for none of its tail.
    bulk_size_table_t t = {0};
    bulk_size_feed(&t, "Blob (bulk_downlink_start_blob) args_str: "
                       "'mpi_data/2026-08-08.mpi;0;0'", T0);
    bulk_size_feed(&t, "[A:LFS:INFO]: Bulk downlink complete. 1000000 bytes "
                       "(5129 packets) downlinked. File: mpi_data/2026-08-08.mpi", T0 + 60000.0);

    int exact = 1;
    long sz = bulk_size_lookup(&t, "mpi_data/2026-08-08.mpi", &exact);
    tap_okf(sz == BULK_SIZE_FIRMWARE_CAP,
            "an uncapped download gives the firmware cap (got %ld)", sz);
    tap_ok(exact == 0, "the firmware cap is a floor, not the file's length");
    bulk_size_free(&t);
}

static void test_one_byte_under_the_cap_is_exact(void)
{
    // The boundary the rule turns on: one byte short of the allowance means the
    // file ended, not the allowance.
    bulk_size_table_t t = {0};
    bulk_size_feed(&t, "Blob (bulk_downlink_start_blob) args_str: 'f;0;0'", T0);
    bulk_size_feed(&t, "Bulk downlink complete. 999999 bytes (1 packets) "
                       "downlinked. File: f", T0 + 1000.0);
    int exact = 0;
    long sz = bulk_size_lookup(&t, "f", &exact);
    tap_okf(sz == 999999 && exact == 1,
            "stopping one byte under the cap is an exact length (got %ld exact %d)", sz, exact);
    bulk_size_free(&t);
}

static void test_exact_beats_a_later_lower_bound(void)
{
    // The real store holds all of these for one file. Whatever order they are
    // read in, the exact 556728 must win over the capped 371085 -- taking the
    // most recent, or the largest without regard to exactness, would not.
    bulk_size_table_t t = {0};
    bulk_size_feed(&t, L_START_ZERO, T0);
    bulk_size_feed(&t, L_COMPLETE_FULL, T0 + 60000.0);
    bulk_size_feed(&t, L_START_CAPPED, T0 + 120000.0);
    bulk_size_feed(&t, L_COMPLETE_CAPPED, T0 + 180000.0);

    int exact = 0;
    long sz = bulk_size_lookup(&t, MPI_PATH, &exact);
    tap_okf(sz == 556728 && exact == 1,
            "an exact length survives a later capped download (got %ld exact %d)", sz, exact);
    bulk_size_free(&t);
}

static void test_stale_start_is_not_paired(void)
{
    // A complete whose own start line was never decoded must not borrow an old
    // start offset: 307125 + 556728 would claim a file nearly twice its length
    // and put the missing list back where it started.
    bulk_size_table_t t = {0};
    bulk_size_feed(&t, L_START_307125, T0);
    bulk_size_feed(&t, L_COMPLETE_FULL, T0 + BULK_SIZE_PAIR_WINDOW_MS + 1000.0);

    long sz = bulk_size_lookup(&t, MPI_PATH, NULL);
    tap_okf(sz == 556728,
            "a start too old to belong is ignored (got %ld, want 556728)", sz);
    bulk_size_free(&t);
}

static void test_paths_do_not_bleed(void)
{
    bulk_size_table_t t = {0};
    bulk_size_feed(&t, "Blob (bulk_downlink_start_blob) args_str: 'ADCS/log_4bfe.TLM;0;0'", T0);
    bulk_size_feed(&t, L_START_ZERO, T0 + 1000.0);
    bulk_size_feed(&t, L_COMPLETE_FULL, T0 + 2000.0);

    tap_okf(bulk_size_lookup(&t, MPI_PATH, NULL) == 556728, "the MPI file has its length");
    tap_okf(bulk_size_lookup(&t, "ADCS/log_4bfe.TLM", NULL) == 0,
            "a file with no complete line has no length");
    tap_okf(bulk_size_lookup(&t, "mpi_data/nothing.mpi", NULL) == 0,
            "an unknown file has no length");
    tap_okf(bulk_size_lookup(&t, "", NULL) == 0, "an empty path has no length");
    bulk_size_free(&t);
}

// ---- the offset grid ---------------------------------------------------

static void test_grid_learns_both_real_downloads(void)
{
    // The shape of the real 2026-07-21 experiment: one download commanded from
    // offset 0 (residue 0), one from 306000 (residue 45), both CRC-verified,
    // plus CRC-failed chunks whose offset fields took bit flips.
    int n = 0;
    long off[600];
    unsigned char ver[600];
    for (int k = 0; k < 400; k++) { off[n] = (long) k * 195;          ver[n] = 1; n++; }
    for (int k = 0; k < 150; k++) { off[n] = 306000 + (long) k * 195; ver[n] = 1; n++; }
    long strays[] = { 132971, 181136, 289027, 328417, 335642, 342343,
                      343114, 344098, 346918, 355333, 669430, 863371, 1024675 };
    int nstray = (int) (sizeof strays / sizeof strays[0]);
    for (int k = 0; k < nstray; k++) { off[n] = strays[k]; ver[n] = 0; n++; }

    unsigned char grid[195];
    bulk_grid_learn(off, ver, n, grid);

    tap_ok(bulk_grid_ok(grid, 0) == 1, "the residue of the offset-0 download is on the grid");
    tap_ok(bulk_grid_ok(grid, 306000) == 1, "the residue of the offset-306000 download is on the grid");

    int accepted_real = 0, accepted_stray = 0;
    for (int i = 0; i < 550; i++) if (bulk_grid_ok(grid, off[i])) accepted_real++;
    for (int i = 550; i < n; i++) if (bulk_grid_ok(grid, off[i])) accepted_stray++;
    tap_okf(accepted_real == 550, "every verified offset is on the grid (%d of 550)", accepted_real);
    tap_okf(accepted_stray == 0, "every bit-flipped offset is off the grid (%d of %d accepted)",
            accepted_stray, nstray);
}

static void test_grid_ignores_unverified_offsets_when_learning(void)
{
    // The whole point of learning from CRC-verified chunks alone: a corrupted
    // offset must not teach the grid its own residue and let itself back in.
    long off[60];
    unsigned char ver[60];
    for (int k = 0; k < 59; k++) { off[k] = (long) k * 195; ver[k] = 1; }   // residue 0
    off[59] = 123457; ver[59] = 0;                                          // residue 82
    unsigned char grid[195];
    bulk_grid_learn(off, ver, 60, grid);
    tap_ok(bulk_grid_ok(grid, 123457) == 0,
           "an unverified offset does not put its own residue on the grid");
    tap_ok(bulk_grid_ok(grid, 195 * 700) == 1, "the verified residue is still accepted");
}

static void test_grid_admits_a_stray_landing_on_a_real_residue(void)
{
    // Honest about the limit: a flipped offset that happens to land on a
    // residue the download really used cannot be told apart this way, and is
    // accepted. Pinned so a future change to the rule is a deliberate one.
    long off[100];
    unsigned char ver[100];
    for (int k = 0; k < 99; k++) { off[k] = (long) k * 195; ver[k] = 1; }
    off[99] = 195L * 5000; ver[99] = 0;      // residue 0, far past the others
    unsigned char grid[195];
    bulk_grid_learn(off, ver, 100, grid);
    tap_ok(bulk_grid_ok(grid, off[99]) == 1,
           "a stray on a real residue is not caught by the grid rule");
}

static void test_grid_opens_when_too_little_is_verified(void)
{
    // A pass that verified almost nothing cannot teach a grid. Enforcing one
    // learned from a handful of chunks would throw away real data, so the grid
    // opens instead.
    long off[30];
    unsigned char ver[30] = {0};
    for (int k = 0; k < 30; k++) off[k] = (long) k * 195;
    for (int k = 0; k < BULK_GRID_MIN_VERIFIED - 1; k++) ver[k] = 1;
    unsigned char grid[195];
    bulk_grid_learn(off, ver, 30, grid);
    tap_ok(bulk_grid_ok(grid, 123457) == 1,
           "below the verified-chunk floor the grid accepts anything");

    // One more verified chunk is enough to start enforcing it.
    ver[BULK_GRID_MIN_VERIFIED - 1] = 1;
    bulk_grid_learn(off, ver, 30, grid);
    tap_ok(bulk_grid_ok(grid, 123457) == 0,
           "at the floor the learned grid is enforced");
}

static void test_grid_rejects_a_negative_offset(void)
{
    long off[30];
    unsigned char ver[30];
    for (int k = 0; k < 30; k++) { off[k] = (long) k * 195; ver[k] = 1; }
    unsigned char grid[195];
    bulk_grid_learn(off, ver, 30, grid);
    tap_ok(bulk_grid_ok(grid, -1) == 0, "a negative offset is never on the grid");
}

int main(void)
{
    test_parse_complete();
    test_parse_complete_rejects();
    test_parse_start();
    test_parse_start_rejects();
    test_size_from_full_download();
    test_size_from_partial_download();
    test_capped_download_is_only_a_lower_bound();
    test_firmware_cap_is_not_a_length();
    test_one_byte_under_the_cap_is_exact();
    test_exact_beats_a_later_lower_bound();
    test_stale_start_is_not_paired();
    test_paths_do_not_bleed();
    test_grid_learns_both_real_downloads();
    test_grid_ignores_unverified_offsets_when_learning();
    test_grid_admits_a_stray_landing_on_a_real_residue();
    test_grid_opens_when_too_little_is_verified();
    test_grid_rejects_a_negative_offset();
    return tap_done();
}

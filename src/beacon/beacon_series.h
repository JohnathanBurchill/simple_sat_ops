/*

    Simple Satellite Operations  beacon_series.h

    The beacons as numbers over time: a catalogue of every telemetry
    field a FrontierSat beacon carries, each with where it sits in the
    packet, what it means, and how to turn its raw bytes into a value in
    engineering units.

    Why a table rather than a struct read: a time-series browser wants
    to offer the operator a list of fields to plot, by name, and to pull
    one field out of forty thousand packets without a switch statement
    per field. So the layout knowledge lives here once, as data, and
    beacon_series_value is the only code that reads a packet.

    The offsets are the ones in beacon_cts1.h's two packed structs, and
    the _Static_asserts in beacon_series.c tie each one to its struct
    member so a firmware resync cannot silently move a field out from
    under the table.

    The basic beacon and the extended (blob) one share their first 130
    bytes field for field, so most of the catalogue applies to both; the
    entries past that are marked as the extended beacon's alone.

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

#ifndef BEACON_SERIES_H
#define BEACON_SERIES_H

#include <stddef.h>
#include <stdint.h>

// How a field's bytes are laid out in the packet. Every multi-byte
// field is little-endian: the satellite is an STM32 and the firmware
// writes its structs straight to the wire.
typedef enum {
    BS_U8, BS_I8, BS_U16, BS_I16, BS_U32, BS_I32,

    // Fields that are not a plain integer at an offset. Each is worked
    // out in beacon_series_value.
    BS_MAG_MAGNITUDE,      // the magnetic field vector's length
    BS_ADCS_EST_MODE,      // estimation mode, out of the packed state
    BS_ADCS_CTRL_MODE,     // control mode, likewise
    BS_ADCS_RUN_MODE,      // run mode, likewise
    BS_ADCS_SUN_UP,        // the Sun-above-the-horizon flag
    BS_ADCS_FAULT_COUNT,   // how many of the forty fault flags are set
} bs_type_t;

// The values a field takes when the satellite had no reading to put
// there. Each is a different subsystem's idea of "not available", and
// plotting any of them as a number puts a spike through the middle of
// an otherwise readable series.
typedef enum {
    BS_SENT_NONE = 0,
    BS_SENT_EPS_I16,    // -9999, the extended-beacon blob's placeholder
    BS_SENT_EPS_I32,    // -99999, the same for its 32-bit fields
    BS_SENT_TEMP_I16,   // a dead thermistor's INT16_MAX or INT16_MIN, or
                        // the blob's own -9999 for a battery temperature
    BS_SENT_TEMP_I32,   // INT16_MAX / INT16_MIN, 32-bit
    BS_SENT_MPI_TEMP,   // -99 inactive, -98 and -97 error codes
    BS_SENT_PACK,       // 0xFFFF, the battery-pack status when unread

    // Zero, where the satellite reports zero for a quantity that cannot
    // be zero while it is still talking to us. The battery voltage is
    // the one that matters: the EPS fails to answer its housekeeping
    // query often -- on two beacons in three across the record -- and
    // both the flight firmware and the blob leave the voltage and the
    // charge percentage at zero when it does. Plotted, that is a curve
    // that spends most of its time on the floor.
    BS_SENT_ZERO,

    // -1, a count that cannot be negative. The blob's placeholder for
    // an EPS fault count it could not read.
    BS_SENT_NEG1,

    // 255 in a one-byte enum, the blob's placeholder for an EPS mode it
    // could not read.
    BS_SENT_U8_MAX,
} bs_sentinel_t;

// What has to be true of the packet before a field means anything.
typedef enum {
    BS_NEEDS_NOTHING = 0,
    BS_NEEDS_ADCS,       // the ADCS answered its query at all
    BS_NEEDS_ATTITUDE,   // and was in an estimation mode that fills the angles
} bs_needs_t;

// Which panel a field belongs under in a browser's field list. Purely
// for grouping the list; nothing depends on the numbering.
typedef enum {
    BS_GROUP_POWER = 0,
    BS_GROUP_THERMAL,
    BS_GROUP_SOLAR,
    BS_GROUP_HOUSEKEEPING,
    BS_GROUP_ADCS,
    BS_GROUP_ATTITUDE,
    BS_GROUP_N
} bs_group_t;

typedef struct {
    const char   *key;        // short stable name, for a flag or a CSV header
    const char   *label;      // what a plot's heading says
    const char   *unit;       // "V", "C", "deg/s", or "" for a count
    unsigned      off;        // byte offset into the packet
    bs_type_t     type;
    double        scale;      // raw value times this gives the unit
    int           ext_only;   // 1 = the extended beacon carries it, not the basic one
    bs_sentinel_t sentinel;
    bs_needs_t    needs;
    bs_group_t    group;
} bs_field_t;

// The catalogue, and its length.
extern const bs_field_t BEACON_SERIES_FIELDS[];
extern const size_t     BEACON_SERIES_FIELD_N;

const char *beacon_series_group_name(bs_group_t g);

// Pull one field out of one beacon. payload / len are a whole beacon
// packet, basic or extended -- the function works out which. Writes the
// value in the field's own units to *out and returns 1; returns 0 and
// leaves *out alone when this packet does not carry the field, or
// carries a sentinel in place of a reading, or has not met the field's
// precondition.
int beacon_series_value(const bs_field_t *f,
                        const uint8_t *payload, size_t len,
                        double *out);

// Find a field by key. NULL if there is no such field.
const bs_field_t *beacon_series_find(const char *key);

#endif // BEACON_SERIES_H

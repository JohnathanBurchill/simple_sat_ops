/*

    Simple Satellite Operations  tcmd_spec.h

    The set of telecommands the satellite accepts, mirrored from the flight
    firmware so the ground software can lint an agenda before it goes on the
    air: each command's name, the exact number of comma-separated arguments it
    expects, and its readiness (risk) level.

    The data table itself (tcmd_spec.c) is GENERATED from the firmware's
    TCMD_telecommand_definitions[] by scripts/gen_tcmd_spec.py and checked in,
    so the ground build never needs the firmware repo. This header is the
    stable hand-written interface over it.

    Copyright (C) 2026  Johnathan K Burchill

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

#ifndef TCMD_SPEC_H
#define TCMD_SPEC_H

#include <stddef.h>
#include <string.h>

// Telecommand readiness / risk level, mirrored from the flight firmware's
// TCMD_readiness_level_enum_t. The numeric values match the firmware so the
// meaning is unambiguous; only TCMD_READY_OPERATION is meant for routine
// flight use.
typedef enum {
    TCMD_READY_OPERATION          = 0,   // normal operation in flight
    TCMD_READY_RECOVERY_OR_EXPERT = 10,  // flight-safe but expert-only
    TCMD_READY_FLIGHT_TESTING     = 20,  // flight-safe but disruptive (e.g. flash test)
    TCMD_READY_GROUND_ONLY        = 30,  // umbilical / ground use only
    TCMD_READY_HIGH_RISK          = 40,  // high risk / unsafe
} tcmd_readiness_t;

// Per-argument type, mirrored from which TCMD_extract_*_arg helper the
// firmware callback uses for that argument. The ground linter uses these to
// reject an argument the firmware's parser would itself reject (so we don't
// burn an uplink on a command that can't parse). One char per argument:
//
//   'u'  uint64    -- digits only, 1..19 of them (TCMD_extract_uint64_arg)
//   'i'  int64     -- optional leading '-', then digits (TCMD_extract_int64_arg)
//   'd'  double    -- optional '-', digits, at most one '.', not at either
//                     end, no exponent (TCMD_extract_double_arg)
//   'h'  hex bytes -- hex digits; ' ' or '_' allowed only between whole bytes
//                     (TCMD_extract_hex_array_arg)
//   'b'  base64    -- base64 alphabet + '=' + URL-safe '-'/'_'; ' ' allowed
//                     only between whole quartets (TCMD_extract_base64_array_arg)
//   's'  string / free-form -- accepted as-is. Covers TCMD_extract_string_arg
//                     (which trims surrounding whitespace), whole-args strings,
//                     enum-by-name args (e.g. "nominal"), and atoi/strtol-parsed
//                     numerics -- none of which the parser rejects on format.
//   '?'  unknown   -- type could not be determined; the linter skips arg
//                     checks for it (fail-open). Should not appear for a
//                     verified table, but is honored so a future regen that
//                     drops this column degrades safely rather than mis-flags.
//
// arg_types is exactly num_args characters long (empty string for 0-arg
// commands). The linter validates only argument *format* at the parser level;
// it does NOT check command-specific value domains (e.g. which enum strings a
// command accepts) -- the firmware rejects those at execution, not at parse.
typedef enum {
    TCMD_ARG_UINT64 = 'u',
    TCMD_ARG_INT64  = 'i',
    TCMD_ARG_DOUBLE = 'd',
    TCMD_ARG_HEX    = 'h',
    TCMD_ARG_BASE64 = 'b',
    TCMD_ARG_STRING = 's',
    TCMD_ARG_UNKNOWN = '?',
} tcmd_arg_type_t;

// One telecommand the firmware accepts.
typedef struct {
    const char      *name;       // exact, case-sensitive command name
    int              num_args;   // required count of comma-separated args
    tcmd_readiness_t readiness;  // risk level
    const char      *arg_types;  // one type code per arg (see above); never NULL
} tcmd_spec_t;

// The generated table (tcmd_spec.c) and its size.
extern const tcmd_spec_t TCMD_SPEC[];
extern const size_t      TCMD_SPEC_COUNT;
// Firmware tag the table was generated from (e.g. "sat-1-rc3").
extern const char *const TCMD_SPEC_FW_TAG;

// Human label for a readiness level (for messages).
static inline const char *tcmd_readiness_label(tcmd_readiness_t r)
{
    switch (r) {
        case TCMD_READY_OPERATION:          return "operation";
        case TCMD_READY_RECOVERY_OR_EXPERT: return "recovery/expert";
        case TCMD_READY_FLIGHT_TESTING:     return "flight-testing";
        case TCMD_READY_GROUND_ONLY:        return "ground-only";
        case TCMD_READY_HIGH_RISK:          return "high-risk/unsafe";
    }
    return "unknown";
}

// Find a telecommand by name, matching the firmware exactly: case-sensitive,
// full-length. `name_len` lets the caller pass an unterminated slice (the
// name part of a "CTS1+name(...)" string); pass strlen(name) for a C string.
// Returns NULL if no command of that exact name exists.
static inline const tcmd_spec_t *tcmd_spec_find(const char *name, size_t name_len)
{
    for (size_t i = 0; i < TCMD_SPEC_COUNT; ++i) {
        const char *cand = TCMD_SPEC[i].name;
        if (strlen(cand) == name_len && strncmp(cand, name, name_len) == 0) {
            return &TCMD_SPEC[i];
        }
    }
    return NULL;
}

#endif // TCMD_SPEC_H

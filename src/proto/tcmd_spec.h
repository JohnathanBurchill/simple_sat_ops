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

// One telecommand the firmware accepts.
typedef struct {
    const char      *name;       // exact, case-sensitive command name
    int              num_args;   // required count of comma-separated args
    tcmd_readiness_t readiness;  // risk level
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

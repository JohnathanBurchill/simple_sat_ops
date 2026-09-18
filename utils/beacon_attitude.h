/*

    Simple Satellite Operations  utils/beacon_attitude.h

    Which way the satellite was facing at a given moment, out of the
    extended beacons in the packet database.

    The extended-beacon blob downlinks the ADCS's estimated attitude
    angles every half minute or so while it is running, and the ADCS
    fills those angles in four of its eight estimation modes. So the
    record is a scatter of moments rather than a continuous history:
    this loads every one that carries a usable attitude, and answers
    "what was the nearest one to this moment, and how far away was it".

    Used by the two raylib viewers to draw the pointing direction over
    the globe -- frontiersat_camera_viewer for the moment a picture was
    taken, mpi_viewer for wherever the playback head is in a recording.

    Timekeeping: the moments are ground reception times
    (packet.ts_received), not the satellite's own clock. The beacon
    carries that clock too, but it runs off the EPS real-time clock and
    drifts -- tens of seconds at a time have been seen -- whereas the
    ground clock is good to well under a second and is what everything
    else in these viewers is keyed on.

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

#ifndef BEACON_ATTITUDE_H
#define BEACON_ATTITUDE_H

#include "sat_globe.h"

// How far from a moment a beacon may sit and still be taken to say
// where the satellite was looking. The satellite turns at a few tenths
// of a degree a second at most in the modes that estimate an attitude,
// so two minutes is a handful of degrees -- worth showing, and the
// caption says how old the beacon was whenever it is not close.
#define BEACON_ATTITUDE_MAX_AGE_S 120.0

// Read every extended beacon with a usable attitude out of the packet
// database, ordered in time. Returns how many were found, or -1 if the
// database could not be read. Calling it again reloads, which is what
// a viewer's reload key should do.
int beacon_attitude_load(const char *db_path);

// The attitude nearest unix_ms, if one lies within max_age_s of it.
// Fills *out and returns 1; returns 0 and zeroes *out when there is
// none, which is the case a viewer draws as no pointing direction at
// all.
int beacon_attitude_nearest(double unix_ms, double max_age_s,
                            globe_attitude_t *out);

// How many attitudes are loaded, for a viewer that wants to say so.
int beacon_attitude_count(void);

void beacon_attitude_free(void);

#endif // BEACON_ATTITUDE_H

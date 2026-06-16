/*

    Simple Satellite Operations  unit_tests/ipc_fill_selftest.c

    Pins ipc_fill_state_prediction() — the prediction -> STATE field
    mapping shared by the operator broadcast (control/operator_ipc.c) and
    the headless --viewer-stream relay. A drift here would silently
    mis-render a viewer's sky position / pass timing, so this proves a
    known prediction_t lands in the right STATE fields, that the fields it
    must NOT touch (the source tag, rotator az/el, has_rotator) are left
    alone, and that the filled event still encodes.

    Links only ipc_fill.c + the codec — no socket, no ncurses, no SGP4.

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

#include "ipc_fill.h"
#include "sso_ipc.h"
#include "tap.h"

#include <string.h>

int main(void)
{
    prediction_t p = {0};
    p.satellite_ephem.name = "FRONTIERSAT";
    snprintf(p.satellite_ephem.tle.idesg, sizeof p.satellite_ephem.tle.idesg,
             "98067A");
    p.minutes_since_epoch                   = 42.5;
    p.predicted_minutes_until_visible       = 12.25;
    p.predicted_minutes_above_0_degrees     = 9.5;
    p.predicted_minutes_above_30_degrees    = 3.0;
    p.predicted_max_elevation               = 71.5;
    p.satellite_ephem.azimuth               = 123.25;
    p.satellite_ephem.elevation             = -4.5;
    p.satellite_ephem.altitude_km           = 512.0;
    p.satellite_ephem.latitude              = 50.8688;
    p.satellite_ephem.longitude             = -114.291;
    p.satellite_ephem.speed_km_s            = 7.61;
    p.satellite_ephem.range_km              = 1840.0;
    p.satellite_ephem.range_rate_km_s       = -3.25;

    sso_event_t e;
    sso_event_init(&e, SSO_EVT_STATE);
    // Pre-seed the fields the helper must leave untouched, so a stray
    // write shows up.
    snprintf(e.source, sizeof e.source, "operator");
    e.az = 200.0; e.el = 45.0; e.has_rotator = 1;

    ipc_fill_state_prediction(&p, &e);

    tap_ok(strcmp(e.satellite, "FRONTIERSAT") == 0, "satellite name mapped");
    tap_ok(strcmp(e.idesg, "98067A") == 0, "idesg mapped");
    tap_ok(e.epoch_min == 42.5, "epoch_min mapped");
    tap_ok(e.min_visible == 12.25, "min_visible mapped");
    tap_ok(e.min_above_0 == 9.5, "min_above_0 mapped");
    tap_ok(e.min_above_30 == 3.0, "min_above_30 mapped");
    tap_ok(e.max_el == 71.5, "max_el mapped");
    tap_ok(e.pred_az == 123.25 && e.pred_el == -4.5, "pred az/el mapped");
    tap_ok(e.alt_km == 512.0, "alt_km mapped");
    tap_ok(e.lat_deg == 50.8688 && e.lon_deg == -114.291, "lat/lon mapped");
    tap_ok(e.speed_kms == 7.61, "speed_kms mapped");
    tap_ok(e.range_km == 1840.0, "range_km mapped");
    tap_ok(e.range_rate_kms == -3.25, "range_rate_kms mapped");

    // Fields the helper must NOT touch.
    tap_ok(strcmp(e.source, "operator") == 0, "source left untouched");
    tap_ok(e.az == 200.0 && e.el == 45.0, "rotator az/el left untouched");
    tap_ok(e.has_rotator == 1, "has_rotator left untouched");

    // The filled event still encodes cleanly and carries the satellite.
    char line[4096];
    e.has_state = 1;
    tap_ok(sso_event_encode(&e, line, sizeof line) == 0, "filled event encodes");
    tap_ok(strstr(line, "\"sat\":\"FRONTIERSAT\"") != NULL,
           "encoded line carries the satellite");

    // NULL-safety: must not crash, must not write.
    sso_event_t z = {0};
    ipc_fill_state_prediction(NULL, &z);
    ipc_fill_state_prediction(&p, NULL);
    tap_ok(z.satellite[0] == '\0', "NULL prediction leaves event empty");

    return tap_done();
}

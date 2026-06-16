/*

   Simple Satellite Operations  ipc/ipc_fill.c

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

#include "ipc_fill.h"

#include <stdio.h>

void ipc_fill_state_prediction(const prediction_t *p, sso_event_t *evt)
{
    if (p == NULL || evt == NULL) return;
    if (p->satellite_ephem.name) {
        snprintf(evt->satellite, sizeof evt->satellite, "%s",
                 p->satellite_ephem.name);
    }
    snprintf(evt->idesg, sizeof evt->idesg, "%s",
             p->satellite_ephem.tle.idesg);
    evt->epoch_min      = p->minutes_since_epoch;
    evt->min_visible    = p->predicted_minutes_until_visible;
    evt->min_above_0    = p->predicted_minutes_above_0_degrees;
    evt->min_above_30   = p->predicted_minutes_above_30_degrees;
    evt->max_el         = p->predicted_max_elevation;
    evt->pred_az        = p->satellite_ephem.azimuth;
    evt->pred_el        = p->satellite_ephem.elevation;
    evt->alt_km         = p->satellite_ephem.altitude_km;
    evt->lat_deg        = p->satellite_ephem.latitude;
    evt->lon_deg        = p->satellite_ephem.longitude;
    evt->speed_kms      = p->satellite_ephem.speed_km_s;
    evt->range_km       = p->satellite_ephem.range_km;
    evt->range_rate_kms = p->satellite_ephem.range_rate_km_s;
}

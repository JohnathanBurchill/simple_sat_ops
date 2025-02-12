/*

    Simple Satellite Operations  prediction.h

    Copyright (C) 2025  Johnathan K Burchill

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
#ifndef PREDICTION_H
#define PREDICTION_H

#include "state.h"

/* RAO site observer location in Priddis, SW of Calgary */
#define RAO_LATITUDE  50.8688  // Latitude in degrees
#define RAO_LONGITUDE -114.2910 // Longitude in degrees
#define RAO_ALTITUDE  1279.0   // Altitude in meters

void update_satellite_position(state_t *state, double jul_utc);
void update_doppler_shifted_frequencies(state_t *state, double uplink_freq, double downlink_freq);
void update_pass_predictions(state_t *external_state, double jul_utc_start, double delta_t_minutes);
void minutes_until_visible(state_t *external_state, double delta_t_minutes, double max_minutes);
int load_tle(state_t *state);


#endif // PREDICTION_H
      

/*

   Simple Satellite Operations  radio_backend.h

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

#ifndef RADIO_BACKEND_H
#define RADIO_BACKEND_H

#include <stdint.h>

// Forward declaration. radio_t is defined in radio.h; radio.h includes this
// file, so we cannot include radio.h here without a cycle.
struct radio;
typedef struct radio radio_t;

// Selectable radio personalities. Default is ICOM_CIV so binaries compiled
// from this branch behave identically to tx-tone-branch builds when the
// new --radio-type= flag is absent.
typedef enum radio_backend_type {
    RADIO_BACKEND_ICOM_CIV = 0,
    RADIO_BACKEND_YAESU_CAT,
    RADIO_BACKEND_USRP_B210,
    RADIO_BACKEND__COUNT,
} radio_backend_type_t;

// Vtable for the operations radio.h exposes. NULL pointer = "this backend
// does not support this operation"; the public dispatcher in radio.c
// returns RADIO_NOT_SUPPORTED with a one-line stderr warning so callers
// can continue without bespoke per-backend branching.
typedef struct radio_backend_ops {
    const char *name;

    int    (*init)(radio_t *r);
    void   (*disconnect)(radio_t *r);

    // Raw protocol escape hatch. Only the IC-9700 backend implements this;
    // it stays in the public surface because main.c uses it directly for
    // scope/waterfall streaming, which is IC-9700-specific anyway.
    int    (*command)(radio_t *r, uint8_t cmd, int16_t subcmd, int16_t subsubcmd,
                      uint8_t *data, int len, uint64_t *return_value, uint8_t reverse_value);

    int    (*set_vfo)(radio_t *r, int vfo);
    double (*get_frequency)(radio_t *r);
    int    (*set_frequency)(radio_t *r, double hz);
    int    (*get_satellite_mode)(radio_t *r);
    int    (*set_satellite_mode)(radio_t *r, int on);
    int    (*set_mode)(radio_t *r, int mode, int filter);
    int    (*set_data_mode)(radio_t *r, int on, int filter);
    int    (*set_data_mod_source)(radio_t *r, int source);
    int    (*set_usb_mod_level)(radio_t *r, int level_0_to_255);
    int    (*set_moni_level)(radio_t *r, int level_0_to_255);
    int    (*set_rf_power)(radio_t *r, int level_0_to_255);
    int    (*ptt)(radio_t *r, int on);
    int    (*get_band_selection)(radio_t *r, int band);
    int    (*set_band_selection)(radio_t *r, int band);
    int    (*toggle_waterfall)(radio_t *r);
} radio_backend_ops_t;

// Wires r->ops to the chosen personality. Call once after zero-initialising
// the radio_t and before radio_init(). Returns 0 on success, non-zero if the
// requested backend was not compiled in (e.g. WITH_USRP_B210=OFF).
int radio_backend_select(radio_t *r, radio_backend_type_t type);

// Look up backend type by short string ("icom-civ", "yaesu-cat", "usrp-b210").
// Returns RADIO_BACKEND__COUNT (== invalid) if unknown.
radio_backend_type_t radio_backend_type_from_string(const char *s);

// Per-backend ops getters. Each returns a pointer to a const static struct
// in its own translation unit, or NULL if that backend was not compiled in.
const radio_backend_ops_t *radio_backend_icom_civ_ops(void);
const radio_backend_ops_t *radio_backend_yaesu_cat_ops(void);
const radio_backend_ops_t *radio_backend_usrp_b210_ops(void);

#endif // !RADIO_BACKEND_H

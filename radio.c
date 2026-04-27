/*

   Simple Satellite Operations  radio.c

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

// Public radio API. Pure dispatcher: every function looks up an op on
// radio->ops (a backend-specific vtable) and calls it. Wire protocol,
// per-radio quirks and BCD packing live in radio_icom_civ.c,
// radio_yaesu_cat.c and radio_usrp_b210.c respectively. Callers don't
// know which radio they're driving.

#include "radio.h"
#include "radio_backend.h"

#include <stdio.h>
#include <string.h>

// Default backend = IC-9700. radio_init() applies this if no caller has
// run radio_backend_select(); preserves byte-for-byte behaviour for code
// paths that don't yet pass --radio-type=.
static const radio_backend_ops_t *default_ops_or_icom(const radio_backend_ops_t *ops)
{
    return ops ? ops : radio_backend_icom_civ_ops();
}

// One-line warning when a caller hits an op the active backend doesn't
// implement. Centralised so the message is uniform.
static int unsupported(const radio_t *r, const char *op)
{
    const char *backend = (r && r->ops && r->ops->name) ? r->ops->name : "(unset)";
    fprintf(stderr, "radio: op '%s' not supported by backend '%s'\n", op, backend);
    return RADIO_NOT_SUPPORTED;
}

int radio_backend_select(radio_t *r, radio_backend_type_t type)
{
    if (r == NULL) return RADIO_ERROR;
    const radio_backend_ops_t *ops = NULL;
    switch (type) {
        case RADIO_BACKEND_ICOM_CIV:  ops = radio_backend_icom_civ_ops();  break;
        case RADIO_BACKEND_YAESU_CAT: ops = radio_backend_yaesu_cat_ops(); break;
        case RADIO_BACKEND_USRP_B210: ops = radio_backend_usrp_b210_ops(); break;
        default: break;
    }
    if (ops == NULL) {
        fprintf(stderr, "radio: backend type %d not available\n", (int)type);
        return RADIO_ERROR;
    }
    r->ops = ops;
    return RADIO_OK;
}

radio_backend_type_t radio_backend_type_from_string(const char *s)
{
    if (s == NULL) return RADIO_BACKEND__COUNT;
    if (strcmp(s, "icom-civ")  == 0) return RADIO_BACKEND_ICOM_CIV;
    if (strcmp(s, "yaesu-cat") == 0) return RADIO_BACKEND_YAESU_CAT;
    if (strcmp(s, "usrp-b210") == 0) return RADIO_BACKEND_USRP_B210;
    return RADIO_BACKEND__COUNT;
}

int radio_init(radio_t *radio)
{
    if (radio == NULL) return RADIO_ERROR;
    radio->ops = default_ops_or_icom(radio->ops);
    if (!radio->ops || !radio->ops->init) return unsupported(radio, "init");
    return radio->ops->init(radio);
}

// radio_connect was historically callable directly. Backends that need a
// serial port open it inside their init(); we no longer expose connect as
// a separate phase, but keep the symbol so any stragglers compile. It's a
// no-op now — removing it would break the ABI of every other caller.
void radio_connect(radio_t *radio)
{
    (void)radio;
}

void radio_disconnect(radio_t *radio)
{
    if (radio == NULL || radio->ops == NULL || radio->ops->disconnect == NULL) return;
    radio->ops->disconnect(radio);
}

int radio_command(radio_t *radio, uint8_t cmd, int16_t subcmd, int16_t subsubcmd,
                  uint8_t *data, int len, uint64_t *return_value, uint8_t reverse_value)
{
    if (!radio || !radio->ops || !radio->ops->command) return unsupported(radio, "command");
    return radio->ops->command(radio, cmd, subcmd, subsubcmd, data, len, return_value, reverse_value);
}

int radio_set_vfo(radio_t *radio, int vfo)
{
    if (!radio || !radio->ops || !radio->ops->set_vfo) return unsupported(radio, "set_vfo");
    return radio->ops->set_vfo(radio, vfo);
}

double radio_get_frequency(radio_t *radio)
{
    if (!radio || !radio->ops || !radio->ops->get_frequency) {
        unsupported(radio, "get_frequency");
        return -1.0;
    }
    return radio->ops->get_frequency(radio);
}

int radio_set_frequency(radio_t *radio, double frequency)
{
    if (!radio || !radio->ops || !radio->ops->set_frequency) return unsupported(radio, "set_frequency");
    return radio->ops->set_frequency(radio, frequency);
}

int radio_get_satellite_mode(radio_t *radio)
{
    if (!radio || !radio->ops || !radio->ops->get_satellite_mode) {
        unsupported(radio, "get_satellite_mode");
        return -1;
    }
    return radio->ops->get_satellite_mode(radio);
}

int radio_set_satellite_mode(radio_t *radio, int sat_mode)
{
    if (!radio || !radio->ops || !radio->ops->set_satellite_mode) return unsupported(radio, "set_satellite_mode");
    return radio->ops->set_satellite_mode(radio, sat_mode);
}

int radio_set_mode(radio_t *radio, int mode, int filter)
{
    if (!radio || !radio->ops || !radio->ops->set_mode) return unsupported(radio, "set_mode");
    return radio->ops->set_mode(radio, mode, filter);
}

int radio_set_data_mode(radio_t *radio, int on, int filter)
{
    if (!radio || !radio->ops || !radio->ops->set_data_mode) return unsupported(radio, "set_data_mode");
    return radio->ops->set_data_mode(radio, on, filter);
}

int radio_set_data_mod_source(radio_t *radio, int source)
{
    if (!radio || !radio->ops || !radio->ops->set_data_mod_source) return unsupported(radio, "set_data_mod_source");
    return radio->ops->set_data_mod_source(radio, source);
}

int radio_set_usb_mod_level(radio_t *radio, int level_0_to_255)
{
    if (!radio || !radio->ops || !radio->ops->set_usb_mod_level) return unsupported(radio, "set_usb_mod_level");
    return radio->ops->set_usb_mod_level(radio, level_0_to_255);
}

int radio_set_moni_level(radio_t *radio, int level_0_to_255)
{
    if (!radio || !radio->ops || !radio->ops->set_moni_level) return unsupported(radio, "set_moni_level");
    return radio->ops->set_moni_level(radio, level_0_to_255);
}

int radio_set_rf_power(radio_t *radio, int level_0_to_255)
{
    if (!radio || !radio->ops || !radio->ops->set_rf_power) return unsupported(radio, "set_rf_power");
    return radio->ops->set_rf_power(radio, level_0_to_255);
}

// Composition of three primitives. Backends that NULL out set_data_mode
// (e.g. FT-991A, where DATA mode is a mode change rather than a flag) get
// RADIO_NOT_SUPPORTED back; we treat that as OK and keep going so the
// uplink is still configured by the other two ops.
int radio_uplink_prep(radio_t *radio)
{
    int rc = radio_set_mode(radio, RADIO_MODE_FM, RADIO_FILTER_FIL1);
    if (rc != RADIO_OK) return rc;
    rc = radio_set_data_mode(radio, 1, RADIO_FILTER_FIL1);
    if (rc != RADIO_OK && rc != RADIO_NOT_SUPPORTED) return rc;
    rc = radio_set_data_mod_source(radio, RADIO_DATA_MOD_SRC_USB);
    if (rc != RADIO_OK && rc != RADIO_NOT_SUPPORTED) return rc;
    return RADIO_OK;
}

int radio_ptt(radio_t *radio, int on)
{
    // PTT-on is gated by the inhibit flag. PTT-off is always allowed —
    // releasing the radio must never be blocked.
    if (on && !(radio && radio->tx_inhibit_cleared)) {
        fprintf(stderr, "radio: PTT inhibited; pass --allow-tx to enable TX.\n");
        return RADIO_TX_INHIBITED;
    }
    if (!radio || !radio->ops || !radio->ops->ptt) return unsupported(radio, "ptt");
    return radio->ops->ptt(radio, on);
}

int radio_get_band_selection(radio_t *radio, int band)
{
    if (!radio || !radio->ops || !radio->ops->get_band_selection) {
        unsupported(radio, "get_band_selection");
        return -1;
    }
    return radio->ops->get_band_selection(radio, band);
}

int radio_set_band_selection(radio_t *radio, int band)
{
    if (!radio || !radio->ops || !radio->ops->set_band_selection) return unsupported(radio, "set_band_selection");
    return radio->ops->set_band_selection(radio, band);
}

int radio_toggle_waterfall(radio_t *radio)
{
    if (!radio || !radio->ops || !radio->ops->toggle_waterfall) return unsupported(radio, "toggle_waterfall");
    return radio->ops->toggle_waterfall(radio);
}

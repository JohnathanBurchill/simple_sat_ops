/*

   Simple Satellite Operations  sdr_backend.c

   Dispatcher for the pluggable SDR backends. Selects a backend by type
   (or auto-probes), allocates the handle, and forwards every public call
   to the active backend's vtable entry. Mirrors radio.c.

   Copyright (C) 2026  Johnathan K Burchill

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

#include "sdr_backend.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Resolve a type to its ops, honoring which backends were compiled in.
// Returns NULL for an unavailable / unknown backend.
static const sdr_backend_ops_t *ops_for(sdr_backend_type_t t)
{
    switch (t) {
        case SDR_TYPE_UHD:
#ifdef WITH_USRP_B210
            return sdr_backend_uhd_ops();
#else
            return NULL;
#endif
        case SDR_TYPE_RTLSDR:
#ifdef WITH_RTL_SDR
            return sdr_backend_rtlsdr_ops();
#else
            return NULL;
#endif
        default:
            return NULL;
    }
}

int sdr_backend_type_from_string(const char *s, sdr_backend_type_t *out)
{
    if (s == NULL || out == NULL) return -1;
    if (strcmp(s, "auto") == 0)   { *out = SDR_TYPE_AUTO;   return 0; }
    if (strcmp(s, "uhd") == 0)    { *out = SDR_TYPE_UHD;    return 0; }
    if (strcmp(s, "rtlsdr") == 0 || strcmp(s, "rtl-sdr") == 0
        || strcmp(s, "rtl") == 0) { *out = SDR_TYPE_RTLSDR; return 0; }
    return -1;
}

int sdr_backend_open(sdr_backend_type_t type,
                     const sdr_open_params_t *p,
                     sdr_backend_t **out)
{
    if (out == NULL || p == NULL) return -1;
    *out = NULL;

    // Build the try-order: a single explicit type, or the auto-probe
    // sequence (UHD first, then RTL-SDR).
    sdr_backend_type_t order[2];
    int n = 0;
    if (type == SDR_TYPE_AUTO) {
        order[n++] = SDR_TYPE_UHD;
        order[n++] = SDR_TYPE_RTLSDR;
    } else {
        order[n++] = type;
    }

    for (int i = 0; i < n; ++i) {
        const sdr_backend_ops_t *ops = ops_for(order[i]);
        if (ops == NULL || ops->open == NULL) continue;
        sdr_backend_t *be = (sdr_backend_t *)calloc(1, sizeof *be);
        if (be == NULL) {
            fprintf(stderr, "sdr_backend: out of memory\n");
            return -1;
        }
        be->ops = ops;
        // Contract: on failure the backend frees its own priv and leaves
        // be->priv NULL, so freeing the handle here is sufficient.
        if (ops->open(be, p, &be->caps) == 0) {
            *out = be;
            return 0;
        }
        free(be);
    }

    fprintf(stderr, "sdr_backend: no SDR backend could open (requested type %d)\n",
            (int)type);
    return -1;
}

void sdr_backend_close(sdr_backend_t *be)
{
    if (be == NULL) return;
    if (be->ops != NULL && be->ops->close != NULL) be->ops->close(be);
    free(be);
}

ssize_t sdr_backend_read_iq(sdr_backend_t *be, int16_t *out, size_t cap_pairs)
{
    if (be == NULL || be->ops == NULL || be->ops->read_iq == NULL) return -1;
    return be->ops->read_iq(be, out, cap_pairs);
}

int sdr_backend_set_freq(sdr_backend_t *be, double freq_hz)
{
    if (be == NULL || be->ops == NULL || be->ops->set_freq == NULL) return -1;
    return be->ops->set_freq(be, freq_hz);
}

double sdr_backend_get_actual_freq(sdr_backend_t *be)
{
    if (be == NULL || be->ops == NULL || be->ops->get_actual_freq == NULL) return 0.0;
    return be->ops->get_actual_freq(be);
}

int sdr_backend_set_gain(sdr_backend_t *be, double gain_db)
{
    if (be == NULL || be->ops == NULL || be->ops->set_gain == NULL) return -1;
    return be->ops->set_gain(be, gain_db);
}

int sdr_backend_tx_burst(sdr_backend_t *be, const sdr_tx_burst_params_t *p)
{
    if (be == NULL || be->ops == NULL) return -1;
    if (be->ops->tx_burst == NULL) {
        fprintf(stderr, "sdr_backend: TX not supported by backend '%s' (RX-only)\n",
                be->ops->name ? be->ops->name : "?");
        return -1;
    }
    return be->ops->tx_burst(be, p);
}

const sdr_caps_t *sdr_backend_caps(const sdr_backend_t *be)
{
    return be ? &be->caps : NULL;
}

int sdr_backend_can_tx(const sdr_backend_t *be)
{
    return be && be->ops && be->ops->tx_burst != NULL && be->caps.can_tx;
}

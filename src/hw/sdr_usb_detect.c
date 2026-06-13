/*

   Simple Satellite Operations  sdr_usb_detect.c

   See sdr_usb_detect.h.

   Copyright (C) 2026  Johnathan K Burchill

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

#include "sdr_usb_detect.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef HAVE_LIBUSB
#include <libusb.h>
#endif

// Ettus B2xx after the FX3 firmware load. Genuine boards and clones
// share these; only the serial string differs.
#define B2XX_VID 0x2500
#define B2XX_PID 0x0020

int sdr_usb_b2xx_serial(char *out, size_t cap)
{
    if (out == NULL || cap == 0) return -1;
    out[0] = '\0';
#ifdef HAVE_LIBUSB
    libusb_context *ctx = NULL;
    if (libusb_init(&ctx) != 0) return -1;
    libusb_device **list = NULL;
    ssize_t n = libusb_get_device_list(ctx, &list);
    int rc = -1;
    for (ssize_t i = 0; i < n && rc != 0; i++) {
        struct libusb_device_descriptor d;
        if (libusb_get_device_descriptor(list[i], &d) != 0) continue;
        if (d.idVendor != B2XX_VID || d.idProduct != B2XX_PID) continue;
        libusb_device_handle *h = NULL;
        if (libusb_open(list[i], &h) == 0 && h != NULL) {
            unsigned char s[128] = {0};
            if (d.iSerialNumber != 0
                && libusb_get_string_descriptor_ascii(h, d.iSerialNumber,
                                                      s, sizeof s) > 0) {
                snprintf(out, cap, "%s", (const char *)s);
                rc = 0;
            }
            libusb_close(h);
        }
    }
    if (list != NULL) libusb_free_device_list(list, 1);
    libusb_exit(ctx);
    return rc;
#else
    return -1;
#endif
}

// The shared FPGA map: one admin-managed file that serves every operator.
// A B210 clone is identical to a genuine board on the USB bus except for its
// serial number; map that serial to the clone's bitstream here and
// simple_sat_ops loads it automatically. Both this map and the FPGA image it
// points at are supplied out of band by the admin — neither is shipped or
// installed by the build. Hardcoded under the same /usr/local tree the
// binaries install into; overridable at build time (e.g. for a non-/usr/local
// prefix or for tests) via -DSDR_FPGA_MAP_SYSTEM_PATH=...
#ifndef SDR_FPGA_MAP_SYSTEM_PATH
#define SDR_FPGA_MAP_SYSTEM_PATH "/usr/local/share/sso/sdr_fpga_map"
#endif

int sdr_fpga_map_path(char *out, size_t cap)
{
    int n = snprintf(out, cap, "%s", SDR_FPGA_MAP_SYSTEM_PATH);
    return (n > 0 && (size_t)n < cap) ? 0 : -1;
}

// Scan the map file for `serial`, writing the mapped image path to `out`.
// Lines are "<serial> <absolute-image-path>"; '#' and blank lines are
// skipped. Returns 1 on a hit, 0 on miss or an unreadable/absent file.
static int fpga_scan_map_file(const char *path, const char *serial,
                              char *out, size_t cap)
{
    FILE *f = fopen(path, "r");
    if (f == NULL) return 0;
    char line[1024];
    int hit = 0;
    while (fgets(line, sizeof line, f) != NULL) {
        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '#' || *p == '\n' || *p == '\r' || *p == '\0') continue;
        char ser[128] = {0};
        char img[768] = {0};
        if (sscanf(p, "%127s %767s", ser, img) == 2
            && strcmp(ser, serial) == 0) {
            snprintf(out, cap, "%s", img);
            hit = 1;
            break;
        }
    }
    fclose(f);
    return hit;
}

int sdr_fpga_for_serial(const char *serial, char *out, size_t cap)
{
    if (out != NULL && cap > 0) out[0] = '\0';
    if (serial == NULL || serial[0] == '\0' || out == NULL) return 0;
    return fpga_scan_map_file(SDR_FPGA_MAP_SYSTEM_PATH, serial, out, cap);
}

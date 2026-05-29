/*

   Simple Satellite Operations  sdr_usb_detect.h

   Identify a USRP B2xx by its USB serial, and map that serial to an
   FPGA image. A B210 clone is byte-identical to a genuine board on the
   USB bus (same VID 0x2500, PID 0x0020, product string "USRP B200") -
   the serial number is the ONLY distinguishing field, and it is
   readable at enumeration before any FPGA is loaded. UHD's own
   uhd_usrp_find() can't be used for this (it segfaults on macOS), so we
   read the descriptor directly via libusb.

   Copyright (C) 2026  Johnathan K Burchill

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

#ifndef SDR_USB_DETECT_H
#define SDR_USB_DETECT_H

#include <stddef.h>

// Read the USB serial of the first attached Ettus B2xx (VID 0x2500,
// PID 0x0020) via libusb, without loading firmware/FPGA. Returns 0 and
// fills out on success; -1 if none found or libusb is unavailable
// (built without HAVE_LIBUSB).
int sdr_usb_b2xx_serial(char *out, size_t cap);

// Resolve the FPGA image mapped to a USB serial in the per-host map
// file (see sdr_fpga_map_path). Lines are "<serial> <absolute-path>";
// '#' and blank lines are ignored. Returns 1 + path on a hit, 0 on a
// miss.
int sdr_fpga_for_serial(const char *serial, char *out, size_t cap);

// Absolute path of the map file
// (~/.local/share/simple_sat_ops/sdr_fpga_map). Returns 0 on success.
int sdr_fpga_map_path(char *out, size_t cap);

// Create the map file with a template comment if it doesn't exist yet,
// seeding a commented example line for `serial` so the operator can see
// where to add their clone. Best-effort, no-op if it already exists.
void sdr_fpga_map_ensure_template(const char *serial);

#endif // SDR_USB_DETECT_H

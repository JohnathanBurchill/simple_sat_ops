/*

   Simple Satellite Operations  radio_device_store.h

   Copyright (C) 2026  Johnathan K Burchill

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

// Persistent default for --radio-device=. Stored as a single line in
//   $HOME/.local/share/simple_sat_ops/radio_device
// so the operator doesn't have to retype the device path on every
// invocation, and so a different machine (Mac vs Linux box) can each
// remember its own FT-991A / IC-9700 path.

#ifndef RADIO_DEVICE_STORE_H
#define RADIO_DEVICE_STORE_H

#include <stddef.h>

// Fills `out` with the absolute path of the persistent store file.
// Returns 0 on success, -1 if HOME is unset or the buffer is too small.
int radio_device_store_path(char *out, size_t cap);

// Reads the saved device path into `out`. Returns:
//   0  : a non-empty value was loaded
//  -1  : no stored value (file does not exist or is empty)
//  -2  : I/O / parsing error
int radio_device_store_load(char *out, size_t cap);

// Writes `path` to the store, creating ~/.local/share/simple_sat_ops as
// needed. Returns 0 on success, -1 on error (errno preserved).
int radio_device_store_save(const char *path);

// Same store directory, sibling file `radio_serial_speed`. Holds a single
// decimal integer (bps) on a single line.
int radio_device_store_load_speed(int *out_bps);
int radio_device_store_save_speed(int bps);

#endif // !RADIO_DEVICE_STORE_H

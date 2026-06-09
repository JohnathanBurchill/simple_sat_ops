/*

   Simple Satellite Operations  utils/sdr_probe.c

   Pre-flight SDR probe. Run before simple_sat_ops to confirm which
   device is attached and which antenna ports it will receive and
   transmit on. Reports the USB serial (the only field that tells a
   B210 clone from a genuine board), the FPGA image that will be loaded
   (stock, or a clone image from the serial->image map), then opens the
   device the way simple_sat_ops would and reports its RX/TX antennas.
   Finally lists any RTL-SDR dongles.

   The USB serial is read via libusb (sdr_usb_detect), not
   uhd_usrp_find, which segfaults on macOS UHD 4.10.

   Copyright (C) 2026  Johnathan K Burchill

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "argparse.h"
#include "sdr_usb_detect.h"

#include <uhd.h>
#include <uhd/usrp/usrp.h>
#include <uhd/error.h>

#ifdef WITH_RTL_SDR
#include <rtl-sdr.h>
#endif

// Parsed command-line configuration. parse_args() fills this; main() copies
// the field out into the working local so the probe body is unchanged.
typedef struct {
    const char *user_args;
} sdr_probe_args_t;

// Option column width: the widest label below ("--uhd-args=<args>") + a small
// margin. See src/cli/argparse.h for the parse_args convention.
#define OPTW 19

// Parse argv into *a (help == 0), or print one right-aligned help line per
// option and return (help != 0). Each option is one self-contained block whose
// test carries "|| help", so help mode falls through and prints them all.
static int parse_args(sdr_probe_args_t *a, int argc, char **argv, int help)
{
    int ntokens = help ? 1 : argc - 1;
    for (int t = 0; t < ntokens; ++t) {
        const char *arg = help ? "" : argv[t + 1];
        int matched = 0;

        if (strcmp(arg, "-h") == 0 || strcmp(arg, "--help") == 0 || help) {
            if (help) parse_help_line(OPTW, "-h, --help", "show this help and exit");
            else { parse_args(a, argc, argv, HELP_BRIEF); return PARSE_HELP; }
            matched = 1;
        }
        if (strncmp(arg, "--uhd-args=", 11) == 0 || help) {
            if (help) parse_help_line(OPTW, "--uhd-args=<args>", "UHD device args verbatim; bypasses serial detection and the FPGA map");
            else a->user_args = arg + 11;
            matched = 1;
        }

        if (!matched && !help) {
            fprintf(stderr, "sdr_probe: unknown argument '%s'\n", arg);
            return PARSE_ERROR;
        }
    }
    return PARSE_OK;
}

static void probe_uhd(const char *user_args)
{
    printf("== UHD ==\n");

    char serial[128] = {0};
    char mapimg[768] = {0};
    int have_serial = (sdr_usb_b2xx_serial(serial, sizeof serial) == 0);
    if (have_serial) {
        printf("  USB serial:  %s\n", serial);
        if (sdr_fpga_for_serial(serial, mapimg, sizeof mapimg)) {
            printf("  FPGA image:  %s  (clone image, from sdr_fpga_map)\n",
                   mapimg);
        } else {
            char mp[512] = {0};
            (void)sdr_fpga_map_path(mp, sizeof mp);
            printf("  FPGA image:  stock (serial not in %s)\n", mp);
        }
    } else {
        printf("  USB serial:  no Ettus B2xx on the bus (or libusb absent)\n");
    }

    // Build the args we'll actually open with — mirrors sdr_uhd's
    // resolver: --uhd-args wins, else type=b200 [+serial=] [+fpga=].
    char args[1024];
    if (user_args != NULL && user_args[0]) {
        snprintf(args, sizeof args, "%.1000s", user_args);
    } else {
        int wf = (mapimg[0] != '\0');
        int ws = have_serial;
        snprintf(args, sizeof args, "%.200s%s%.127s%s%.512s",
                 "type=b200",
                 ws ? ",serial=" : "", ws ? serial : "",
                 wf ? ",fpga="   : "", wf ? mapimg : "");
    }

    printf("  opening:     %s\n", args);
    uhd_usrp_handle dev = NULL;
    uhd_error e = uhd_usrp_make(&dev, args);
    if (e != UHD_ERROR_NONE || dev == NULL) {
        char errbuf[256] = {0};
        (void)uhd_get_last_error(errbuf, sizeof errbuf);
        printf("  open failed: %s\n", errbuf[0] ? errbuf : "(not found)");
        return;
    }

    char mb[64] = {0};
    if (uhd_usrp_get_mboard_name(dev, 0, mb, sizeof mb) == UHD_ERROR_NONE && mb[0]) {
        printf("  device:      %s\n", mb);
    }
    size_t nrx = 0, ntx = 0;
    (void)uhd_usrp_get_rx_num_channels(dev, &nrx);
    (void)uhd_usrp_get_tx_num_channels(dev, &ntx);
    char rxant[32] = {0};
    char txant[32] = {0};
    if (nrx > 0) (void)uhd_usrp_get_rx_antenna(dev, 0, rxant, sizeof rxant);
    if (ntx > 0) (void)uhd_usrp_get_tx_antenna(dev, 0, txant, sizeof txant);
    printf("  RX channels: %zu   (chan 0 antenna now: %s)\n",
           nrx, rxant[0] ? rxant : "?");
    printf("  TX channels: %zu   (chan 0 antenna now: %s)\n",
           ntx, txant[0] ? txant : "?");
    printf("\n");
    printf("  simple_sat_ops will RECEIVE on antenna  RX2\n");
    printf("  simple_sat_ops will TRANSMIT on antenna TX/RX\n");
    printf("  (B200 RF-A; the TX subdev is unmapped between bursts so the\n"
           "   transmit LO does not leak into RX2 while receiving.)\n");

    uhd_usrp_free(&dev);
}

static void probe_rtl(void)
{
    printf("\n== RTL-SDR (RX-only) ==\n");
#ifdef WITH_RTL_SDR
    uint32_t n = rtlsdr_get_device_count();
    if (n == 0) {
        printf("  no RTL-SDR devices\n");
        return;
    }
    for (uint32_t i = 0; i < n; i++) {
        const char *nm = rtlsdr_get_device_name(i);
        printf("  [%u] %s\n", i, nm ? nm : "?");
    }
    printf("  RTL-SDR is receive-only (no transmit). AUTO uses it only\n"
           "  when no UHD device opens. Select one with --sdr-device=<index>.\n");
#else
    printf("  (this build has no RTL-SDR support; -DWITH_RTL_SDR=ON to add it)\n");
#endif
}

// -V / --version support (commit baked in at build time).
#include "sso_version.h"

int main(int argc, char **argv)
{
    if (sso_version_handle(argc, argv, "sdr_probe")) return 0;
    sdr_probe_args_t cfg = {
        .user_args = "",
    };
    switch (parse_args(&cfg, argc, argv, HELP_OFF)) {
        case PARSE_HELP:  return 0;
        case PARSE_ERROR: return 1;
    }
    const char *user_args = cfg.user_args;

    probe_uhd(user_args);
    probe_rtl();
    return 0;
}

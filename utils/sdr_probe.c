/*

   Simple Satellite Operations  utils/sdr_probe.c

   Pre-flight SDR probe. Run before simple_sat_ops to confirm which
   device is attached and which antenna ports it will receive and
   transmit on. Opens the UHD device the way simple_sat_ops would
   (type=b200 by default, or --uhd-args=...), reports the board name,
   RX/TX channel counts and current antennas, and the ports
   simple_sat_ops actually uses. Then lists any RTL-SDR dongles.

   Uses uhd_usrp_make (which fails gracefully when no device is present)
   rather than uhd_usrp_find, which segfaults on macOS UHD 4.10.

   Copyright (C) 2026  Johnathan K Burchill

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <uhd.h>
#include <uhd/usrp/usrp.h>
#include <uhd/error.h>

#ifdef WITH_RTL_SDR
#include <rtl-sdr.h>
#endif

static void usage(const char *argv0)
{
    printf(
        "usage: %s [--uhd-args=<args>]\n"
        "\n"
        "Probe the SDR(s) simple_sat_ops would use, without starting a\n"
        "pass. Reports the UHD device and its RX/TX antenna ports, then\n"
        "lists RTL-SDR dongles.\n"
        "\n"
        "  --uhd-args=<args>   UHD device args (default \"type=b200\").\n"
        "                      Use serial=... to pick one of several.\n",
        argv0);
}

static void probe_uhd(const char *args)
{
    printf("== UHD ==\n");
    uhd_usrp_handle dev = NULL;
    uhd_error e = uhd_usrp_make(&dev, args);
    if (e != UHD_ERROR_NONE || dev == NULL) {
        char errbuf[256] = {0};
        (void)uhd_get_last_error(errbuf, sizeof errbuf);
        printf("  no UHD device for args=\"%s\": %s\n",
               args, errbuf[0] ? errbuf : "(not found)");
        printf("  (only the device matching the args is opened; UHD's\n"
               "   enumeration is not used as it is unreliable on macOS.)\n");
        return;
    }

    char mb[64] = {0};
    if (uhd_usrp_get_mboard_name(dev, 0, mb, sizeof mb) == UHD_ERROR_NONE && mb[0]) {
        printf("  device:      %s   (args=\"%s\")\n", mb, args);
    } else {
        printf("  device:      (name unavailable)   (args=\"%s\")\n", args);
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
    printf("  AUTO backend selection uses THIS UHD device.\n");

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

int main(int argc, char **argv)
{
    const char *args = "type=b200";
    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "--uhd-args=", 11) == 0) {
            args = argv[i] + 11;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "sdr_probe: unknown argument '%s'\n", argv[i]);
            usage(argv[0]);
            return 1;
        }
    }

    probe_uhd(args);
    probe_rtl();
    return 0;
}

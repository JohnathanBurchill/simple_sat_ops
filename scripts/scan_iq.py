#!/usr/bin/env python3
"""
scan_iq.py — sliding-window AX100 ASM sync scan over a raw .iq capture.

Single-file Python prototype that mirrors the C decoder pipeline
(modem_iq.c on the coherent-iq-demod branch):

  AGC -> boxcar matched filter on I,Q -> symbol-rate differential phase
       -> 2nd-power bias estimate (removes constant per-symbol bias
          from any residual carrier-frequency offset) -> slicer ->
          AX100 ASM (0x930B51DE) search under both polarities.

Useful when iterating on the demod without rebuilding the C code — the
script reports the best Hamming match for every window, so you can see
exactly which bursts in a capture syncs find and at what timing.

Defaults are tuned for the simple_sat_ops .iq sidecar format (raw
interleaved int16 I,Q at 48 kHz, 9600 baud MSK), which is what the
B210 RX session writes alongside each pass WAV.

Usage:
    scripts/scan_iq.py <capture.iq>
        [--rate=48000] [--baud=9600]
        [--window-s=1.5] [--slide-s=0.5]
        [--sync-threshold=4]
        [--min-ham=0]   # only print syncs with ham<=N

Example:
    scripts/scan_iq.py ~/FrontierSat/RAO/Operations/.../*.iq \
        --window-s=3 --slide-s=0.25 --min-ham=2
"""

import argparse
import sys
import numpy as np


ASM = 0x930B51DE  # AX100 attached sync marker (big-endian)


def find_asm(bits, max_ham):
    """Lowest-Hamming match of ASM in `bits`. Returns (ham, offset) or None."""
    if len(bits) < 32:
        return None
    win = 0
    for k in range(32):
        win = (win << 1) | int(bits[k])
    best = (33, -1)
    h = bin(win ^ ASM).count("1")
    if h <= max_ham:
        best = (h, 0)
    for k in range(32, len(bits)):
        win = ((win << 1) | int(bits[k])) & 0xFFFFFFFF
        h = bin(win ^ ASM).count("1")
        if h <= max_ham and h < best[0]:
            best = (h, k - 31)
            if h == 0:
                break
    return best if best[1] >= 0 else None


def demod_window(z, sps):
    """One 1.5 s-ish IQ window -> sliced bits (both polarities).

    Mirrors modem_iq.c: AGC, boxcar matched filter on I/Q, symbol-rate
    differential phase with 2nd-power bias removal, fixed sps-spaced
    strobing (no M&M — this is the prototype).
    """
    rms = np.sqrt(np.mean(np.abs(z) ** 2)) + 1e-9
    z = z / rms
    # Boxcar MF on I and Q (same as np.convolve over the complex signal).
    zmf = np.convolve(z, np.ones(sps) / sps, mode="valid")
    # Symbol-rate complex differential.
    diff = zmf[sps:] * np.conj(zmf[:-sps])
    # 2nd-power bias estimate: average diff² has phase 2·bias+π,
    # flip sign and half-arg to recover bias mod π.
    d2 = diff ** 2
    bias = np.angle(-np.mean(d2)) / 2.0
    dphi = np.angle(diff * np.exp(-1j * bias))
    # Fixed-rate strobing at sps spacing.
    strobe = dphi[sps::sps]
    bits_norm = (strobe > 0).astype(np.uint8)
    bits_inv  = (strobe < 0).astype(np.uint8)
    return bits_norm, bits_inv


def main(argv):
    p = argparse.ArgumentParser(
        description="Sliding-window AX100 sync scan over a raw .iq file."
    )
    p.add_argument("iq_path")
    p.add_argument("--rate", type=int, default=48000,
                   help="Sample rate (default 48000)")
    p.add_argument("--baud", type=int, default=9600,
                   help="Symbol rate (default 9600)")
    p.add_argument("--window-s", type=float, default=1.5,
                   help="Decode window length in seconds (default 1.5)")
    p.add_argument("--slide-s", type=float, default=0.5,
                   help="Slide between windows in seconds (default 0.5)")
    p.add_argument("--sync-threshold", type=int, default=4,
                   help="Max ASM bit errors to call a sync (default 4)")
    p.add_argument("--min-ham", type=int, default=4,
                   help="Only print syncs with ham<=N (default 4)")
    args = p.parse_args(argv)

    if args.rate % args.baud != 0:
        sys.exit(f"rate ({args.rate}) must be a multiple of baud ({args.baud})")
    sps = args.rate // args.baud

    raw = np.fromfile(args.iq_path, dtype=np.int16)
    if len(raw) % 2 != 0:
        sys.exit("file has an odd int16 count — not interleaved I,Q?")
    iq = raw[0::2].astype(np.float32) + 1j * raw[1::2].astype(np.float32)
    duration_s = len(iq) / args.rate
    print(f"loaded {len(iq)} pairs = {duration_s:.1f} s @ {args.rate} Hz")

    window_n = int(args.window_s * args.rate)
    slide_n  = int(args.slide_s  * args.rate)
    if window_n < 64 * sps or slide_n < 1:
        sys.exit("window-s / slide-s too small")
    n_windows = (len(iq) - window_n) // slide_n + 1

    print(f"scanning {n_windows} windows of {args.window_s:.2f}s @ "
          f"{args.slide_s:.2f}s slide, sync<=ham {args.sync_threshold}")
    print(f"{'t_start':>9}  {'ham':>3}  {'off':>5}  pol")

    found = 0
    for w in range(n_windows):
        i0 = w * slide_n
        z  = iq[i0 : i0 + window_n]
        bn, bi = demod_window(z, sps)
        rn = find_asm(bn, args.sync_threshold)
        ri = find_asm(bi, args.sync_threshold)
        best = None
        pol  = None
        if rn is not None:
            best, pol = rn, "norm"
        if ri is not None and (best is None or ri[0] < best[0]):
            best, pol = ri, "inv"
        if best is None:
            continue
        ham, off = best
        if ham > args.min_ham:
            continue
        t_s = i0 / args.rate
        print(f"  {t_s:7.2f}s  {ham:3d}  {off:5d}  {pol}")
        found += 1

    print(f"\n{found} sync(s) found at ham<={args.min_ham}")


if __name__ == "__main__":
    main(sys.argv[1:])

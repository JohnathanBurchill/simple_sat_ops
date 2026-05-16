# DECODING.md and its figures

The tutorial itself lives in `DECODING.md` here. This README is the
build recipe for the embedded figures, which are driven by the actual C
encoders (`csp.c`, `ax100.c`, `golay24.c`, `rs.c`) two directories up —
change one of those and re-run `make_figures.sh` to regenerate.

## Files

- `gen_figdata.c` — builds a CSP packet (`CSP{5→10,dport=9}` "CTS1 beacon"),
  runs it through `ax100_frame()` with RS+scrambler+Golay+ASM on, then
  MSK-modulates the resulting bits at h=0.5, dev=2400 Hz, baud=9600, fs=96
  kHz. Adds AWGN at a few SNR levels and runs the same FM-discriminator +
  boxcar MF + Gardner TED that `modem_fsk.c` runs live. Dumps `*.dat`
  files that the gnuplot scripts read.
- `fig_*.gp` — gnuplot scripts, one per figure.
- `make_figures.sh` — compile, run, render. Re-runs idempotent.
- `wire_bytes.txt` — annotated hexdump of the actual frame, useful for
  cross-checking with packet captures.

## Build / run

    ./make_figures.sh

Requires gnuplot, a C compiler, and OpenSSL headers (the `ax100.c` HMAC
support pulls in `<openssl/evp.h>` even when HMAC isn't being used).
On macOS with Homebrew that's `brew install openssl@3 gnuplot` — paths
inside `make_figures.sh` follow the standard `/opt/homebrew/opt/openssl@3`
location.

## What each figure shows

| File | Tutorial section |
|------|------------------|
| `fig_msk_trajectory.png`     | §2 — what MSK actually looks like on the wire |
| `fig_discriminator.png`      | §3 — FM-discriminator output at three SNRs |
| `fig_matched_filter.png`     | §4 — boxcar MF cleaning up the discriminator |
| `fig_gardner_scurve.png`     | §5 — Gardner TED S-curve (restoring force) |
| `fig_eye_diagram.png`        | §6 — eye opening at the slicer |
| `fig_asm_correlation.png`    | §7 — sliding 32-bit Hamming distance to the ASM |
| `fig_scrambler_spectrum.png` | §9 — PSD before/after the CCSDS scrambler |
| `fig_rs_cliff.png`           | §10 — RS(255,223) decode probability vs BER |

All synthetic; SNR/BER ranges chosen for pedagogy, not to mimic a specific
real pass. The numerical values (h = 0.5, dev 2400 Hz, sps = 10, ASM
0x930B51DE, sync_max_ham = 4, RS-K = 223, RS-N = 255) all match the live
chain.

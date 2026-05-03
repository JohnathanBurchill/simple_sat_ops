# simple_sat_ops — USAGE

Opinionated how-to for the three workflows `sso` covers today: **pass
prediction**, **live tracking**, and **AX100 uplink composition**. For
the project's state-of-things and roadmap see `STATUS_*.md`; this file
is just the practical commands, tips, and troubleshooting.

---

## Build

```bash
mkdir -p build && cd build && cmake .. && make install
```

Installs `simple_sat_ops`, `next_in_queue`, `lifetime`, `tx_tone`, and
`uplink_test` to `$HOME/bin`. Needs the vendored `sgp4sdp4` library
pre-installed to `/usr/local/`, plus `libncurses-dev`, `libasound2-dev`,
`libssl-dev`, and `libcurl4-openssl-dev`.

---

## Binaries at a glance

| Binary | What for | Needs hardware? |
|---|---|---|
| `simple_sat_ops` | Live pass tracking, Doppler, ncurses UI | optional (`--with-radio` / `--with-rotator`) |
| `next_in_queue` | Batch pass finder over a TLE file | no |
| `lifetime` | Toy orbit-decay estimate | no |
| `tx_tone` | Sine-tone RF output test | radio |
| `uplink_test` | CSP → AX100 frame → WAV; offline byte-compare | no |

---

## Pass prediction (no hardware)

By default every tool reads from
`~/.local/state/simple_sat_ops/active.tle`. Override with `--tle=<path>`
per invocation to point at a specific file (e.g. `TLEs/amateur.tle`).

**Next ISS pass (default TLE):**
```bash
next_in_queue 0 2000 "ISS (ZARYA)"
```
The positional `<satellite_name>` is a literal, case-sensitive **prefix**
match. Drop it to scan every satellite.

**All passes in the next 3 hours above 30° elevation:**
```bash
next_in_queue 0 2000 --list --max-minutes=180 --min-elevation=30
```

**Filter by regex against a specific TLE:**
```bash
next_in_queue 0 2000 --tle=TLEs/amateur.tle \
  --list --regex='ISS|ZARYA' --ignore-case
```

**Constellation swarms** (Starlink / OneWeb / Flock / Iridium / etc.)
are filtered out by default — they'd otherwise flood the list with
hundreds of look-alike entries. Pass `--all-satellites` to include them:
```bash
next_in_queue 0 2000 --list --all-satellites
```

**Annotate with radio info.** `--show-radio-info` reads an
`active_radios.txt` file sitting **next to whichever TLE is in use** —
so with the default TLE that means
`~/.local/state/simple_sat_ops/active_radios.txt`; with
`--tle=TLEs/amateur.tle` it's `TLEs/active_radios.txt`. If the file
isn't there, missing annotations are just omitted.
```bash
next_in_queue 0 2000 --list --show-radio-info
```

### TLE source

Grab a fresh Celestrak catalog and drop it into the default location:
```bash
mkdir -p ~/.local/state/simple_sat_ops
wget -O ~/.local/state/simple_sat_ops/active.tle \
  "https://celestrak.org/NORAD/elements/gp.php?GROUP=active&FORMAT=tle"
```
Celestrak serves CRLF-terminated files; sso's parser handles that.

### SSM trajectory source (for FrontierSat)

Once an object is propagated and uploaded via
[`space_safety_manager`](../space_safety_manager), `next_in_queue` can
plan passes directly against that trajectory — no TLE needed.

```bash
# List available trajectories (JSON)
ssm trajectories

# Plan passes using one (note: no positional min/max altitudes)
next_in_queue --trajectory-id=<uuid> --list

# Specific receive-only window
next_in_queue --trajectory-id=<uuid> --list \
  --max-minutes=180 --min-elevation=20
```

Gotchas:
- `ssm` must be on `PATH` (installed to `$HOME/bin/ssm` by its build).
- The trajectory has a finite propagated window (defaults to 3 h when
  created by `ssm propagate ... --duration 3`). Inside the window,
  state is cubic-Hermite interpolated from the OEM samples. Beyond
  the window, `next_in_queue` falls back to **two-body Kepler
  extrapolation** from the last OEM sample: no J2, no drag. For a
  LEO that's ~minutes/day of AOS drift — fine for ground-segment
  scheduling, not fine for space-safety / conjunction screening.
  A note is printed on stderr when `--max-minutes` would push past
  the window.
- Not compatible with `--tle=` — they're mutually exclusive sources.
- If you want higher-accuracy long-horizon predictions, regenerate the
  trajectory with a longer duration (`ssm propagate <opm> --duration
  168 ...` for one week) so the full span is inside the Hermite-
  interpolated window.

---

## Live tracking (with hardware)

```bash
# Full setup: IC-9700 over native USB, SPID rotator, auto-select next pass
simple_sat_ops next --with-hardware \
  --radio-device=/dev/ttyACM0 --min-elevation=10
```

**Keyboard is locked by default.** Press `K` to unlock for ~5 s, then
one of:
- `T` — start tracking
- `s` — stop
- `r` — park (az=0, el=0)
- `[` / `]` — nudge azimuth ±5°
- `q` — quit

---

## AX100 uplink (CSP → AX100 frame → audio)

End-to-end status: framer + modulator verified byte-exact against
`pycsplink`, live-ALSA streaming not yet wired in. For today, the
workflow is "generate WAV offline, play through the 9700" as the
integration test. M3 will replace the aplay step with sso keying PTT
and streaming the PCM directly.

### 1. Keyfile (one-time)

sso expects the HMAC key at `~/.local/state/simple_sat_ops/frontiersat_hmac`.
Plain uppercase hex, no spaces, no separators, one trailing newline
allowed. `chmod 0600` is mandatory — sso refuses the file otherwise.

```bash
mkdir -p ~/.local/state/simple_sat_ops
# Paste / echo your hex key into the file:
echo -n "00112233445566778899AABBCCDDEEFF" \
  > ~/.local/state/simple_sat_ops/frontiersat_hmac
chmod 600 ~/.local/state/simple_sat_ops/frontiersat_hmac
```

If your existing key is base64, convert once with `base64 -d | xxd -p -u`.

### 2. IC-9700 one-time commissioning

These never change session-to-session; do them at the front panel once:

- `SET > Connectors > USB (B)/DATA Function = CI-V` — routes CI-V over
  USB so sso can control the radio.
- `SET > Connectors > CI-V > USB Port Function` must match
  `--radio-serial-speed` (or be on CDC-ACM, which ignores baud).
- **Dualwatch ON** (so both Main and Sub are addressable via CI-V). The
  `[M/S]` key toggles which band is *active*; the touchscreen's
  `DUAL WATCH` label (or a long-press on `[M/S]`) toggles the feature
  itself. If Dualwatch is off, `0x07 0xD1 (Select Sub)` NGs and sso
  will log a warning + skip the Sub-park step (init still completes).
- Recommended starting VFO state: FM, UHF band on Main, frequency set
  anywhere in 435–438 MHz. sso will override mode/frequency on init.

Everything else (FM-DATA mode, DATA MOD source = USB, USB MOD Level)
is CI-V-reachable and handled by `--uplink-ready`.

### 3. Per-session: prep the radio

```bash
# Configures FM + DATA-on + DATA MOD = USB at startup.
# Add --uplink-mod-level=<0..255> to also set USB MOD Level.
simple_sat_ops next --with-radio \
  --uplink-ready --uplink-mod-level=128
```

Confirm on the front panel after init: the mode display should read
**FM-D** and `SET > Connectors > MOD Input > DATA MOD` should show
**USB**.

### 4. Byte-level verification (offline)

Before trusting any on-air TX, confirm the framer matches the `pycsp` /
`pycsplink` reference exactly:

```bash
# Generate the frame bytes with sso's C framer:
uplink_test --payload-hex=70696e67 \
  --src=1 --dst=2 --dport=3 --sport=4 --prio=2 --print-frame

# Generate the same frame with the Python reference:
CTS_PATH=~/src/CTS_SAT_1_COMMUNICATIONS/Working_Python_Files \
  python3 scripts/verify_uplink_frame.py

# Diff should be empty:
diff <(uplink_test --payload-hex=70696e67 \
         --src=1 --dst=2 --dport=3 --sport=4 --prio=2 --print-frame) \
     <(CTS_PATH=~/src/CTS_SAT_1_COMMUNICATIONS/Working_Python_Files \
         python3 scripts/verify_uplink_frame.py)
```

Empty diff = framer is bit-identical to the authoritative reference.

### 5. Manual audio test (stopgap for M3)

```bash
# 1. Generate a frame WAV (48 kHz mono 16-bit, Gaussian-shaped 9600 bps)
uplink_test --payload-hex=70696e67 --out=/tmp/ping.wav

# 2. Figure out which ALSA card the 9700 is
aplay -l  # look for "IC-9700" or similar; note hw:X,0

# 3. Keep the radio in FM-DATA USB, push audio, key PTT
simple_sat_ops "ISS (ZARYA)" --with-radio --uplink-ready &
# ... trigger PTT via radio button or second CI-V call ...
aplay -D plughw:X,0 /tmp/ping.wav
```

Listen on a second receiver (or the 9700 through a dummy-load coupler)
— you should hear an FSK warble, not chopped voice. On a waterfall the
frame should show up as two tones roughly 3 kHz apart.

---

## Troubleshooting

**`Invalid TLE` errors on Celestrak files.** Confirmed handled as of
`55e8745` / `f7a10a0`. If you still see it on a custom TLE, check for
truncated lines or exotic line endings with `file` and `xxd | head`.

**`Satellite not found` but I can see it in the TLE.** Name matching is
literal-prefix, case-sensitive. `ISS` matches every ISS-family entry;
`iss` matches nothing. Quote names with spaces or parens:
`"ISS (ZARYA)"`.

**`hmac_keyfile: ... must be chmod 0600`.** Strict on purpose.
`chmod 600 ~/.local/state/simple_sat_ops/frontiersat_hmac`.

**CI-V open fails with permission denied.** Your user is probably
not in the `dialout` group. `sudo usermod -aG dialout $USER` then log
out and back in.

**Can't find the 9700 ALSA card.** `aplay -l` lists cards by name;
look for `IC-9700`. If it's not there, verify the USB cable, check
`dmesg | tail` after plug-in, and confirm `SET > Connectors > USB (B)/
DATA Function = CI-V` *plus* that the audio CODEC endpoint is enabled
on the radio's USB (B) menu.

**Rotator reset (`r`) slews unsafely from high elevation.** Known
limitation (`STATUS` P2 #10). Stop tracking first, bring elevation down
manually, then reset.

**Merge opens editor on `git pull`.** Local branch diverged from
remote. `git config --global pull.ff only` to refuse non-fast-forward
pulls; then resolve explicitly (`git rebase origin/<branch>` or
`git merge --no-edit`).

---

## Future work pointers

- Live ALSA streaming, PTT auto-key, and `current_telecommand.txt`
  watcher are **M3** — not wired yet; see `STATUS_*.md` P0 #3/#4.
- 19200 bps experiments need a rational-rate resampler in `modem.c`
  (STATUS P2 #15).
- A separate-SDR TX chain (bypasses the 9700 audio-bandwidth ceiling)
  is on the architectural options list but gated on hardware + license.

When any of these change, update the newest-dated `STATUS_YYYYMMDDa.md`
snapshot rather than editing this file; `USAGE.md` is the stable
how-to, `STATUS` is the moving target.

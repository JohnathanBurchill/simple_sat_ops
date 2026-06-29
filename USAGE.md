# simple_sat_ops — USAGE

The authoritative, tool-by-tool usage guide now lives in
**[`docs/USER_MANUAL.md`](docs/USER_MANUAL.md)** — pass prediction, live
tracking, the SDR RX/decode chain, AX100 uplink composition, the operator
keybindings, and troubleshooting. Start there.

> This page used to carry its own how-to, but it described the retired
> CAT-controlled-radio path (IC-9700 / FT-991A, ALSA audio, `tx_tone`,
> `--with-radio`). The RF chain is SDR-only now, so that material was removed
> rather than left to mislead. See `docs/USER_MANUAL.md` for the current
> workflows and `README.md` for the project overview.

## Quick start

```bash
mkdir -p build && cd build && cmake .. && make install

# List upcoming passes for the ground station (no hardware)
next_in_queue TLEs/amateur.tle 0 2000 --list

# Live UI: pick a satellite by name prefix, or let it choose the next pass
simple_sat_ops TLEs/amateur.tle "ISS (ZARYA)"
simple_sat_ops TLEs/amateur.tle next --min-elevation=10
```

Build prerequisites and options are covered in `README.md` and
`docs/USER_MANUAL.md`.

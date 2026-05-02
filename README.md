# Simple Satellite Operations tools

A set of tools dedicated to predicting and tracking satellite passes near a
ground station, with pluggable radio backends (Yaesu FT-991A by default,
ICOM IC-9700 as a secondary, USRP B210 stub) and a SPIG Rot2Prog antenna
rotator controller.

The aim is twofold: 1) demonstrate reliable and fast operation through a
simple terminal interface, and 2) learn by doing. The software suits needs;
your mileage may vary. 

Directly based on the [sgp4sdp4 library](https://github.com/KJ7LNW/sgp4sdp4)
ported to C by Neoklis Kyriazis from other sources. Thank you
[@KJ7LNW](https://github.com/KJ7LNW) for making this available on github.

**CAUTION**: the original sgp4sdp4 COPYING file is dated 2001. We have not
checked whether this version of sgp4sdp4 is still consistent with that
recommended by [Valado et al.
2006](https://celestrak.org/publications/AIAA/2006-6753/AIAA-2006-6753-Rev3.pdf).

Thanks go to the University of Calgary [Rothney Astrophysical
Observatory](https://science.ucalgary.ca/rothney-observatory) staff and the
[CalgaryToSpace](https://www.calgarytospace.ca) RF communications lead (as of
Feb 2025) for logistics and technical support while testing this software on
the FrontierSat ground station equipment.

Satellite radio data as of 4 March 2025 courtesy [JE9PEL](http://www.ne.jp/asahi/hamradio/je9pel/satslist.csv).

## simple_sat_ops

Custom drivers control the SPIG Rot2Prog antenna rotator and the configured
transceiver for satellite communications over UHF and VHF. The radio path
is dispatched through a backend abstraction (`radio.c` →
`radio_yaesu_cat.c` / `radio_icom_civ.c` / `radio_usrp_b210.c`), so the same
binaries drive any of the supported rigs without forking the code.

![A radio demo gif](demo/simple_sat_ops_demo_radio_only_20250217.gif)

## Radios and TX safety

Pick the radio with `--radio-type=yaesu-cat | icom-civ | usrp-b210` (default
`yaesu-cat`). Pick the modulator audio input with `--mod-input=usb | acc |
mic | ...` (default `acc`, which routes audio from the rear DATA jack on
both Yaesu and ICOM rigs).

Persistent device defaults live at:

```
~/.local/share/simple_sat_ops/radio_device
~/.local/share/simple_sat_ops/radio_serial_speed
```

Set them once with the `radio_ctl` CLI:

```bash
radio_ctl --radio-type=yaesu-cat --radio-device=/dev/ttyUSB1 \
          --radio-serial-speed=38400 --store-device --store-serial-speed identify
```

After that, plain `radio_ctl ...` / `tx_tone ...` / `simple_sat_ops ...`
invocations pick the device and baud up automatically.

TX is **inhibited by default** on every binary so refactor work and bring-up
runs can't accidentally key the PA. Three opt-in gates:

- `--allow-tx` — clears the inhibit. Without it the tool configures the
  radio (frequency, mode, MOD source, power) but stops at PTT.
- `--allow-high-power` — required for `--tx-power` above 10%.
- `--allow-hf-tx` — required to PTT below 100 MHz.

PTT-off is always passed through, even with the inhibit set; releasing TX
is never blocked.

For Yaesu FT-991A operators: see the front-panel one-time-setup checklist
at the top of `radio_yaesu_cat.c` (Menus 031 / 033 / 071 / 072 are
operator-set; 070 / 079 are pinned by CAT during `radio_uplink_prep`).

The ICOM IC-9700 (`icom-civ`) backend compiles and the basic CAT path
works, but it is **untested at satellite-pass level** for FM voice
repeater operation or low-baud data, and the radio appears unable to
handle 9600 baud TX through any audio path tried. For operational
satellite use the Yaesu FT-991A (default `--radio-type=yaesu-cat`) is
the canonical radio.

## next_in_queue

Prints upcoming overpasses of satellites from a file of TLEs using a `sgp4sdp4`
library.

![A demo without hardware gif](demo/simple_sat_ops_demo_no_hardware_20250127.gif)

## lifetime 

Estimates the lifetime of a satellite from a TLE. 

**This is inaccurate** The calculation is a toy 'what if?'. In `sgp4sdp4`, orbit
decay is apparently modelled empirically based on measurement of the rate of
change of the mean anomaly and its rate of change. See the sgp4sdp4 source
code and related references for details.

# Installation

We use ``cmake``:

```bash
mkdir build
cd build
cmake ..
make install
```

This installs the binaries to ``$HOME/bin``. Adapt CMakeLists.txt to suit your needs.

Use whatever build system you prefer, or use one-liners like this:

```bash
gcc -o simple_sat_ops main.c prediction.c radio.c antenna_rotator.c -lncurses -lsgp4sdp4
gcc -o next_in_queue next_in_queue.c prediction.c -lsgp4sdp4
gcc -o lifetime lifetime.c prediction.c -lsgp4sdp4
```

The ```sgp4sdp4``` library needs to be compiled separately and installed in a
suitable location.

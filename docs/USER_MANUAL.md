---
pagetitle: "The Sat Ops Guide"
---

# The CalgaryToSpace Guide to Cubesat Operations

*A field guide to pointing antennas, pulling frames out of the noise,
and talking to a satellite that only answers when you ask politely.*

Version: 2

Prepared by Johnathan K. Burchill and Claude Opus 4.8 at the University
of Calgary.

A tool-by-tool reference for the Calgary-to-Space FrontierSat ground
station software (`simple_sat_ops` and friends), written for the team
members who run passes and the developers who extend the code.

---

## Foreword

Operating a satellite pass combines three pleasures: aiming a few
kilograms of aluminum at a fast-moving point in the sky, coaxing a
coherent frame out of a hiss of thermal noise, and the small thrill of
sending a command that the spacecraft actually answers.

The most important thing to understand is that none of this is hard.
A pass lasts a few minutes. The hardware is forgiving. The software
refuses, by default, to do anything that could hurt the radio, the
rotator, or the link: the transmitter starts inhibited, the rotator
moves only when you tell it to, and only one operator can touch the
hardware at a time. A beginner sitting in viewer mode cannot break
anything, no matter which keys they lean on. The skill is almost all
understanding and only a little practice, and this manual is mostly
about the understanding.

Every part of the ground station lies to you in its own small,
well-understood way, and the job is mostly knowing how. The SPID rotator will
cheerfully report that it has reached a new position the instant you
ask it to move, long before the motor has turned. The B210 will leak
its local oscillator straight out the antenna if you let the transmit
chain idle hot. The satellite answers only the frames that carry the
right signature and drops the rest without a word, so a wrong key
looks exactly like a dead pass. None of these is a defect you fix;
each is a quirk you learn, plan around, and eventually stop thinking
about.

One more thing before the controls: this is a team activity, and the
software is built around that fact. One person runs the pass while
others watch the same screen in viewer mode, calling out a drifting
azimuth, a missed frame, a Doppler curve that looks wrong. Passes are
short and a lot happens at once; a second pair of eyes catches what
the operator, head down in the panels, will miss. If you ever find
yourself running a pass entirely alone, at night, talking to a
spacecraft that only answers when you ask it politely - pause and
reconsider some of your life choices. Then call a teammate.

And one standing request while you read. This manual was written
alongside the code, and the code keeps moving - a flag gets renamed, a
path moves, a default changes - so some of what follows is already
wrong by the time it reaches you. Treat that as part of the job: as you
read, your one task is to catch the errors and report the corrections
as you find them. When the manual and the code disagree, the code
wins (see [How to read this manual](#how-to-read-this-manual)); when
you win an argument with the manual, fix the manual. A guide is only as
honest as its last careful reader left it.

Read the theory before you touch the keys. Then run a pass as a
viewer, where nothing you do reaches the hardware. Then take the
controls. By the third pass the sequence will feel obvious, and this
manual can go back on the shelf where it belongs.

---

## Table of Contents

- [Foreword](#foreword)
- [How to read this manual](#how-to-read-this-manual)

1. [Overview](#overview)
2. [How the downlink works: from photon to plaintext](#how-the-downlink-works-from-photon-to-plaintext)
   - [From radio wave to numbers](#from-radio-wave-to-numbers)
   - [Chasing the Doppler](#chasing-the-doppler)
   - [What's on the wire: MSK](#whats-on-the-wire-msk)
   - [Turning frequency into bits](#turning-frequency-into-bits)
   - [Finding the frame: the sync marker](#finding-the-frame-the-sync-marker)
   - [The length, well protected: Golay(24,12)](#the-length-well-protected-golay2412)
   - [Undoing the scrambler: CCSDS](#undoing-the-scrambler-ccsds)
   - [Correcting the errors: Reed-Solomon(255,223)](#correcting-the-errors-reed-solomon255223)
   - [Reading the packet: CSP](#reading-the-packet-csp)
   - [Watching it happen](#watching-it-happen)
   - [The uplink, in reverse](#the-uplink-in-reverse)
   - [When the chain breaks](#when-the-chain-breaks)
3. [Hardware](#hardware)
4. [Build and install](#build-and-install)
5. [First-run setup](#first-run-setup)
6. [Before you operate: licensing and authorization](#before-you-operate-licensing-and-authorization)
7. [A map of the cat: the tools](#a-map-of-the-cat-the-tools)
8. [Operator UI: `simple_sat_ops`](#operator-ui-simple_sat_ops)
   - [Modes: operator vs. viewer](#modes-operator-vs-viewer)
   - [Command-line options](#command-line-options)
   - [SDR backends](#sdr-backends)
   - [Keyboard controls](#keyboard-controls)
   - [Status, RX, and TX panels](#status-rx-and-tx-panels)
   - [Colon-command prompt](#colon-command-prompt)
   - [TX compose modal (`t`)](#tx-compose-modal-t)
   - [Auto-telecommand modal (`A`)](#auto-telecommand-modal-a)
   - [Finding telecommands and their arguments](#finding-telecommands-and-their-arguments)
   - [Rotator calibration (`--calibrate-rotator`)](#rotator-calibration---calibrate-rotator)
   - [Pursuit tracking](#pursuit-tracking)
9. [Pass scheduling: `next_in_queue`](#pass-scheduling-next_in_queue)
10. [Agenda review: `agenda_check`](#agenda-review-agenda_check)
11. [Offline analysis tools](#offline-analysis-tools)
    - [`gen_waterfall`](#gen_waterfall)
    - [`rx_replay`](#rx_replay)
    - [`decode_inspector`](#decode_inspector)
    - [Replaying SatNOGS recordings](#replaying-satnogs-recordings)
    - [`beacon_detect`](#beacon_detect)
    - [`fm_preview`](#fm_preview)
    - [`packet_query` and `packet_browser`](#packet_query-and-packet_browser)
12. [Bring-up and test tools](#bring-up-and-test-tools)
    - [`tx_frame_sdr`](#tx_frame_sdr)
    - [`b210_rx_capture` and `b210_gain_sweep`](#b210_rx_capture-and-b210_gain_sweep)
    - [`live_waterfall`](#live_waterfall)
    - [`uplink_test`](#uplink_test)
    - [`rx_decode`](#rx_decode)
    - [`lifetime`](#lifetime)
13. [Unit tests](#unit-tests)
14. [Architecture notes](#architecture-notes)
    - [IPC: one operator at a time](#ipc-one-operator-at-a-time)
    - [Worker threads](#worker-threads)
    - [TX safety gates](#tx-safety-gates)
    - [Pursuit planner internals](#pursuit-planner-internals)
15. [File layout](#file-layout)
16. [Troubleshooting](#troubleshooting)
17. [A note on feel](#a-note-on-feel)
18. [Glossary](#glossary)

*Appendices:*

- [Appendix A: What didn't stick - the FT-991A and IC-9700 radio path](#appendix-a-what-didnt-stick---the-ft-991a-and-ic-9700-radio-path)
- [Appendix B: Trial and error - the git repository](#appendix-b-trial-and-error---the-git-repository)

---

## How to read this manual

The chapters run theory first, hardware in the middle, tools last, so
you can read straight through or jump to the tool you need. Each tool
section says what the thing is for before it lists the flags, on the
theory that one flag you understand is worth ten you have memorized.

There is a Feynman story behind that. As a student he was handed a
paper on nerve impulses in cats, thick with the names of muscles - the
extensors, the flexors, the gastrocnemius - and he had no idea where
any of them sat on the animal. So he asked the biology librarian for
"a map of the cat" and got laughed at: the term of art was "zoological
chart," and for a while he was the dumb student who wanted a map of a
cat. But when his turn came to give the talk he drew the outline and
named the muscles off it anyway, and when the other students cut in
with "we know all that," he realized they had spent four years
memorizing what he had just looked up in fifteen minutes.

That is how this manual is meant to be used. You do not need to carry
the flags, the menu numbers, or the frame format around in your head -
you need to know they exist and where to find them when the moment
comes. Read it once for the shape of the thing; after that it is a
reference you look things up in, not a text you memorize.

A few conventions:

| You see | It means |
|---------|----------|
| `monospace` | A literal command, file path, flag, or key to press. |
| `$x^\circ$` | Mathematics, set with LaTeX (degrees, deltas, and the like). This manual uses no special characters outside math, so it survives copy-paste into a wiki, a terminal, or a plain-text email intact. |
| **Drill** | A hands-on exercise. Do each one on real hardware at least once; they build the feel the prose cannot. |
| **Safety** | A point where the software is protecting you from yourself. Read these even if you skip everything else. |

When the manual and the code disagree, the code wins. `--help-full`
and the source files named in each section are authoritative; a manual
is only a snapshot of how things stood when it was written, and
hardware and firmware drift.

---

## Overview

The ground station drives a USRP B210 software-defined radio and a
SPID Rot2ProG antenna rotator to receive telemetry from and command
CalgaryToSpace's CTS-SAT-1 (FrontierSat). Uplink and downlink share
one carrier at **436.150 MHz** (simplex). Downlink is **FSK 9600
baud** with the AX100 frame format: 32-bit ASM, Golay-coded length,
CCSDS scrambling, Reed-Solomon FEC. Every uplink frame carries an
HMAC. The satellite drops unsigned frames silently, so a missing
key is fatal to the link.

A word on why a software-defined radio sits at the centre of this. When
the team came together, few of us held the advanced amateur
qualification needed to put hardware of our own design on the air, and
none of us wanted the project's schedule to depend on earning it. So
the guiding rule was to lean on commercial, off-the-shelf parts
wherever we could: a stock SDR, a stock rotator, stock transceivers
before them. The B210 is COTS in exactly that sense - a finished,
type-approved radio we drive with software rather than a circuit we
built. The advanced work lives in the code, where a mistake costs a
rebuild and not a violation.

There is one **operator UI** binary (`simple_sat_ops`) that owns the
hardware during a pass. Around it sits a set of CLI tools for pass
scheduling, calibration, command-list review, and offline analysis
of recorded captures. The operator UI runs a cooperative
single-threaded ncurses loop but pushes serial I/O, SDR I/O, and
audit-log writes onto background pthreads so device latency never
freezes the screen.

Only **one** operator process can hold the hardware at a time.
Other invocations are *viewers*: they connect to the operator's IPC
socket and render the same state read-only. A viewer can force-claim
the operator role; the running operator yields and the viewer
re-execs itself as the new operator.

## How the downlink works: from photon to plaintext

This is the "how it works" chapter. You can run a pass without reading
it, the way you can drive a car without knowing what the engine does. But
knowing what each stage does is exactly what lets you tell a weak pass
from a broken one, and read the panels instead of just watching them.
Skip the equations on a first pass if you like; the prose carries the
argument and the math is there for when you want it.

Decoding is the satellite's encoding run backwards. To get a few
fragile bytes of telemetry across a few thousand kilometres of noisy
vacuum with a Doppler-smeared carrier, the spacecraft wraps them in
layer after layer of protection. The receiver peels those layers off
in the reverse order. Every arrow below is one small algorithm, and
each was forced on us either by the channel (noise, Doppler, an
unknown symbol clock) or by an upstream encoder that traded raw data
rate for the ability to survive the trip:

```
RF wave at ~436 MHz
  |  antenna, analog frontend, ADC, I/Q mixer
  v
complex baseband samples (96 kHz)
  |  FM discriminator
  v
instantaneous-frequency stream
  |  matched filter
  v
soft symbol estimates
  |  timing recovery + slicer
  v
raw bits
  |  sync-word (ASM) search
  v
framed bits
  |  Golay(24,12) length decode
  v
known-length bytes
  |  CCSDS descrambler
  v
de-whitened bytes
  |  Reed-Solomon(255,223) decode
  v
error-corrected payload
  |  CSP packet parse
  v
header fields + application bytes ("Hello from CalgaryToSpace FrontierSat")
```

![The receive chain, end to end](figures/rx-signal-chain.png)

The rest of this chapter walks down that chain. Everything here runs
live inside `simple_sat_ops` and offline in `rx_replay`; you can watch
it happen stage by stage in `decode_inspector` (see
[Watching it happen](#watching-it-happen)).

### From radio wave to numbers

The satellite transmits a passband signal centred at
$f_c \approx 436.150$ MHz. The B210 tunes to $f_c$ and uses an analog
quadrature mixer to multiply the antenna voltage against both a cosine
and a sine at $f_c$, low-pass filtering each. Pack the two outputs into
one complex number $s(t) = I(t) + i\,Q(t)$, and a sinusoid at
$f_c + \Delta f$ becomes a complex exponential
$e^{\,i\,2\pi\,\Delta f\,t}$ at baseband: a positive $\Delta f$ sits
above the tuner, a negative one below. This is a lock-in amplifier with
two phases.

The B210 hands the host complex IQ at 480 kHz; a software FIR in
`src/dsp/fir_decim.c` decimates by 5 to 96 kHz. At 9600 baud that is
**10 samples per symbol**, comfortable headroom for everything below.

### Chasing the Doppler

The satellite closes on the line of sight at up to
$v_r \approx 7.6$ km/s, shifting the carrier by

$$
\Delta f_{\mathrm{dop}} \approx \pm \frac{v_r}{c}\, f_c
  \approx \pm 10\ \mathrm{kHz}\ \text{at 436 MHz}.
$$

Left alone that would walk the signal clean out of the matched
filter's band by mid-pass. Instead the receiver recomputes the
expected Doppler from the SGP4 propagator every tick and retunes the
B210 whenever the prediction has drifted more than 200 Hz from the
current tune. Near closest approach, where the Doppler rate peaks
around 50 Hz/s, retunes fire every few seconds; far from it, rarely.

The chain downstream is built so a retune is a non-event: the FM
discriminator (below) is differential, so the absolute tuner phase
that jumps at each retune cancels; the residual offset between retunes
is a small DC pedestal that the high-pass filter flattens in about a
millisecond; and a frame lasts about 50 ms while retunes are seconds
apart, so even a frame that straddles a retune loses at most one
sample, well inside the error budget. The rest of this chapter assumes
the tuner stays within $\pm 200$ Hz of the real carrier.

![A pass on the waterfall, Doppler curving the carrier](figures/pass-waterfall-doppler.png)

### What's on the wire: MSK

FrontierSat uses **binary frequency-shift keying**: a `1` is a tone at
$+2400$ Hz relative to the carrier for one symbol, a `0` is a tone at
$-2400$ Hz, at 9600 baud. The dimensionless modulation index is

$$
h = \frac{2 \cdot \mathrm{deviation}}{\mathrm{baud}}
  = \frac{2 \cdot 2400}{9600} = 0.5 .
$$

An index of $h = 0.5$ is **MSK**, minimum-shift keying: the narrowest
FSK that still keeps the two tones orthogonal over a symbol while
holding the phase continuous across symbol boundaries, so it does not
splatter energy into the neighbours. It is the standard setting for the
GomSpace AX100 modem the satellite carries.

Why frequency keying at all? Because the bit lives in the frequency,
not the amplitude, the link shrugs off fading and lets the spacecraft's
power amplifier run hard into saturation without corrupting data. The
price is bandwidth: MSK is wider than an equivalent phase scheme.

### Turning frequency into bits

Four small steps take the complex baseband stream to a clean run of
bits.

**The FM discriminator** turns frequency into a voltage. A clean sample
is $s[n] = A\,e^{\,i\,\phi[n]}$, and the instantaneous frequency is the
phase step, which we read off as

$$
\Delta\phi[n] = \arg\!\bigl(s[n]\,s^{*}[n-1]\bigr).
$$

The product $s[n]\,s^{*}[n-1]$ cancels the absolute phase, so we never
need to know the carrier phase and slow Doppler becomes a constant DC
offset rather than a moving sinusoid. For FrontierSat the output hops
between about $+0.157$ and $-0.157$ rad/sample. Two wrinkles: when
noise dominates, `atan2` wraps and throws a huge spurious spike (the
classic FM "click"), so we low-pass the IQ at 12 kHz first; and the
residual carrier offset puts a DC pedestal on the output that a
one-pole high-pass filter removes.

**The matched filter** averages out the noise. The transmitter holds
each frequency for a full symbol, so the pulse is a rectangle, and the
filter that maximises signal-to-noise at the decision instant is itself
a rectangle: a running mean over the $\mathrm{sps} = 10$ samples of a
symbol. That cuts the noise by $\sqrt{10} \approx 3.2$ while leaving the
signal intact.

**Timing recovery** finds where each symbol actually falls. We have
ten samples per symbol but do not know the phase offset, and the two
clocks drift, so the right sampling instant moves. A Gardner timing
detector, fed fractional-sample values from a Farrow interpolator,
forms an error term

$$
\operatorname{ted}[k] = y\!\left[k-\tfrac{1}{2}\right]\cdot\bigl(y[k]-y[k-1]\bigr),
$$

which is zero at correct timing and changes sign with the direction of
the error; a small loop gain nudges the sampling instant to drive it to
zero.

**The slicer** makes the call: the bit is $1$ when the matched-filter
output is positive, $0$ otherwise. That throws away the soft magnitude,
which a soft-decision decoder could spend to claw back roughly 2 dB,
but the Reed-Solomon layer downstream is strong enough that we do not
need to.

All four live in `src/dsp/modem_fsk.c`.

### Finding the frame: the sync marker

The bit stream is continuous; we have to find where each frame begins.
The transmitter prepends a fixed 32-bit **Attached Sync Marker** to
every frame:

```
ASM = 0x930B51DE
```

We slide a 32-bit window along the bits and count the Hamming distance
to the ASM, accepting any position within 4 bits as a match. The
marker is the CCSDS standard value, chosen for low autocorrelation so a
partial match never masquerades as a full one. With a 4-bit tolerance
the chance a random 32-bit run matches is

$$
P_{\mathrm{fa}} = \frac{1}{2^{32}} \sum_{k=0}^{4} \binom{32}{k}
  \approx 10^{-5},
$$

about one false alarm per pass, which the length, Golay, and RS layers
below reject cheaply.

### The length, well protected: Golay(24,12)

The 24 bits right after the ASM carry the **frame length** in a
Golay(24,12) block code: 12 data bits plus 12 parity, correcting up to
3 bit errors. The length gets the heaviest per-bit protection in the
whole frame for a simple reason: a wrong length offsets the byte
boundary for the entire rest of the frame, and possibly the next one
too. It is the single most expensive field to lose. Lives in
`src/proto/golay24.c`.

### Undoing the scrambler: CCSDS

The payload bytes were XOR'd at the transmitter against a
pseudo-random sequence from an 8-bit shift register seeded to all-ones,
with polynomial

$$
h(x) = x^{8} + x^{7} + x^{5} + x^{3} + 1 ,
$$

the additive scrambler from CCSDS 131.0-B. Descrambling is the same XOR
again. The point is not secrecy (anyone with the standard can undo it)
but engineering hygiene: it guarantees a healthy density of bit
transitions no matter how repetitive the payload, which keeps the
spectrum flat and the timing loop locked. A long run of identical bytes
would otherwise hold the discriminator flat and let the clock drift.

### Correcting the errors: Reed-Solomon(255,223)

The de-whitened payload is a Reed-Solomon codeword: 223 data bytes plus
32 parity bytes, 255 in all, over the finite field
$\mathrm{GF}(2^{8})$ where each byte is one field element. It corrects

$$
t = \left\lfloor \frac{255 - 223}{2} \right\rfloor = 16
$$

byte errors anywhere in the block, about 6% of the codeword. This is
the workhorse: at our usual link margin the slicer makes a handful of
bit errors per frame and RS quietly fixes them.

The interesting part is the failure: the threshold is a cliff, not a
slope. Assuming independent bit errors at rate $p_b$,

$$
P_{\mathrm{ok}}(p_b)
  = \sum_{k=0}^{16} \binom{255}{k}\, p_B^{\,k}\,(1 - p_B)^{\,255-k},
  \qquad p_B = 1 - (1 - p_b)^{8} .
$$

Below about $p_b = 4\times 10^{-3}$ the decoder is essentially
infallible; by $2\times 10^{-2}$ it is hopeless; the transition sits
where the expected byte-error count crosses 16. A frame that fails RS
is rescued only by the partial-decode path in
`src/pipeline/decode_loop.c`, which prints whatever ASCII survived
uncorrected. Lives in `src/proto/rs.c` and `src/proto/ax100.c`.

### Reading the packet: CSP

What survives RS is a **CSP** packet, the CubeSat Space Protocol, in
spirit a stripped-down IP for small satellites. A v1 header is 4 bytes:
priority, source and destination node addresses, source and
destination ports, and a few flags. After it come the application
bytes, an ASCII string for a beacon or a packed binary struct for
telemetry, in the format the flight software defines. The parse lives
in `src/proto/csp.c`; the result is written to the SQLite packet
database (`src/db/packet_db.c`) and broadcast over the IPC bus to any
attached viewers.

### Watching it happen

`decode_inspector` renders this exact chain, stage by stage, on a live
or replayed capture: the ASM correlation dropping to zero at the frame
start, the Golay length decode, the descrambler, the Reed-Solomon
block, and the final CSP parse. It is the best way to build intuition
for the theory above, because you watch a real frame walk through every
step and see which stage a marginal pass dies at.

![decode_inspector walking a frame through the chain](figures/decode-inspector-stages.png)

Where each stage runs in the live receiver:

| Stage | File(s) |
|-------|---------|
| RF capture, downconversion, decimation | `src/hw/b210_rx_tx_core.c`, `src/dsp/fir_decim.c` |
| LPF, FM discriminator, matched filter, timing recovery, slicer, ASM | `src/dsp/modem_fsk.c` |
| MSK Viterbi fallback slicer | `src/dsp/modem_viterbi.c` |
| Length decode, descramble, Reed-Solomon | `src/pipeline/decode_loop.c`, `src/proto/golay24.c`, `src/proto/ax100.c`, `src/proto/rs.c` |
| CSP parse | `src/proto/csp.c` |
| Session glue, WAV capture, dedup, database | `src/pipeline/rx_session.c`, `src/db/packet_db.c` |
| Operator UI and IPC | `apps/main.c` |

For offline work `rx_replay` runs the same chain in two passes: a fast
sync search sweeps the whole capture and collects ASM candidate
timestamps, then the heavier decoder runs on tight windows anchored at
each candidate, which is far quicker than slicing the whole file at
full cost.

### The uplink, in reverse

Transmitting is the same film run backwards. The operator's payload
goes into a CSP header, gets signed with the HMAC the satellite
demands, then runs through Reed-Solomon, the CCSDS scrambler, the Golay
length header, and the ASM, and is MSK-modulated onto the carrier. The
one asymmetry is Doppler: where the receiver chases the incoming shift,
the transmitter must *pre-compensate* the outgoing one, keying at
$f_{\mathrm{carrier}} / (1 - v_r/c)$ so the signal lands on the
satellite's receiver at the right frequency. The mechanics of composing
and sending a burst are in the
[TX compose modal](#tx-compose-modal-t).

### When the chain breaks

The failure modes map onto the stages above, and the operational
symptoms and fixes are collected in [Troubleshooting](#troubleshooting):

* **Doppler tracking lost** - the SGP4 prediction is more than about
  10 kHz off the real carrier, almost always a stale or wrong TLE. The
  signal slides out of the band and the chain never locks. Refresh the
  TLE.
* **Symbol-clock drift beyond about 100 ppm** - the timing loop cannot
  hold; rare with healthy hardware.
* **Modulation-index mismatch** - if the signal is not quite $h = 0.5$,
  the boxcar matched filter is suboptimal and costs around 1 dB. The
  `src/dsp/modem_viterbi.c` slicer recovers most of it on a true MSK
  signal and is the fallback when the discriminator path cannot close.
* **Passes sync and descramble but fail RS** - the link is a few dB
  below the cliff. Nothing to do in software short of soft-decision
  decoding; this is a signal-margin problem, not a decode bug.

## Hardware

| Component | Notes |
|-----------|-------|
| **USRP B210** | Two-channel SDR; channel 0 receives on `RX2` and transmits on `TX/RX`. UHD must be installed. Receive runs continuously, including during a transmit burst. |
| **SPID Rot2ProG** | Az/El antenna rotator with a built-in controller. USB-serial CAT at any baud (it autodetects). Constant slew rates configured on the controller's front panel. |
| **UHF T/R switch** | Optional CalgaryToSpace RP2040-Zero board on USB-CDC at `/dev/ttyACM0`. Senses RF on the TX line and flips two relays (K1 = antenna path, K2 = dummy). Auto-mode by default; the firmware emits a heartbeat the operator UI parses for a status panel. |

The live RF path is the B210. A conventional transceiver (Yaesu
FT-991A, Icom IC-9700) drove that path in an earlier design and the
code to run one is still in the tree, but it is no longer the way we
operate. That history, and the tools that go with it, lives in
[Appendix A](#appendix-a-what-didnt-stick---the-ft-991a-and-ic-9700-radio-path).

## Build and install

CMake, out-of-tree, installs to `$HOME/bin`:

```sh
mkdir -p build && cd build && cmake .. && make install
```

Which targets actually build depends on what the host has:

| Dependency | Targets it unlocks |
|------------|---------------------|
| always | `radio_ctl`, `rs_selftest`, `fm_preview`, `agenda_check` |
| OpenSSL / libcrypto | `uplink_test`, `rx_decode`, `packet_query`, `packet_browser` |
| SGP4SDP4 | `next_in_queue`, `lifetime`, `prediction_selftest`, `pursuit_selftest` |
| UHD (B210) | `b210_rx_capture`, `b210_gain_sweep`, `tx_frame_sdr`, `sdr_probe` |
| librtlsdr | RTL-SDR RX-only backend in `simple_sat_ops` (on by default; auto-disables if absent) |
| libusb | USB-serial clone detection in the UHD backend (a UHD dependency, so normally already present) |
| libsndfile | `rx_replay` reading SatNOGS `.ogg` audio recordings |
| raylib | `live_waterfall`, `decode_inspector` |
| ncurses + SGP4SDP4 + libcrypto | `simple_sat_ops` (pulls UHD too when present) |

### Dependencies

The build auto-detects each library via pkg-config; install the **`-dev`**
packages so the headers and `.pc` files are found.

Ubuntu/Debian (the ground machine):

```sh
sudo apt install build-essential cmake pkg-config \
    libncurses-dev libssl-dev libsqlite3-dev libasound2-dev \
    libuhd-dev librtlsdr-dev libusb-1.0-0-dev libsndfile1-dev
```

macOS (Homebrew, dev host):

```sh
brew install cmake pkg-config ncurses openssl sqlite uhd librtlsdr libusb libsndfile
```

Mind the Debian names: it is **`librtlsdr-dev`** and **`libusb-1.0-0-dev`**
- the bare `librtlsdr` / `libusb` / `libusb-1.0` packages do not exist
(`apt` says "Unable to locate package"). UHD is `libuhd-dev`; libsndfile
(lets `rx_replay` read SatNOGS `.ogg`) is `libsndfile1-dev`; `raylib`
(optional `live_waterfall`) is `libraylib-dev`.

The bundled `sgp4sdp4/` directory is a separate CMake project; build
and install it first (see `sgp4sdp4/README.md`). `CMakeLists.txt`
hard-codes the install location, so a non-default prefix needs a
small edit.

There is no project-wide test suite, linter, or formatter.
`scripts/lint_warnings.sh` runs a parallel gcc-15 build of the whole
tree under `-Wformat-truncation=2 -Werror=format-truncation` to catch
warnings Apple clang misses. Output lives in `build-lint/`.

## First-run setup

1. **Confirm hardware.** Run `sdr_probe` (see
   [SDR backends](#sdr-backends)): it shows the SDR, the FPGA image it
   will load, and the RX/TX antenna ports - and, for a clone, surfaces
   the USB serial even when a wrong-FPGA open would fail. SPID powered
   and in `A` mode on the front panel (the firmware path that responds
   to the W-frame STATUS query). T/R switch USB-CDC visible if
   installed. Antenna connected to the right port.

2. **TLE file.** By convention the team keeps dated TLEs under the
   FrontierSat TLE directory (referred to below as `$TLES`, i.e.
   `$FRONTIERSAT_ROOT/TLEs`), one per day in a date-stamped subfolder:
   `$TLES/20260529/tle-20260529.tle`. `simple_sat_ops --control` with
   no `<satellite_id>` loads the newest `*.tle` it finds there
   (searched recursively) and pins it into the pass folder. Otherwise
   pass `--tle <path>` explicitly, or stage one at the default
   `$HOME/.local/state/simple_sat_ops/active.tle`, which `next_in_queue`
   shares.

3. **HMAC key.** Required for any on-air uplink. The operator UI
   loads the shared keyfile (`/FrontierSat/HMAC/frontiersat_hmac`),
   failing that `$HOME/.local/state/simple_sat_ops/frontiersat_hmac`,
   failing that `--hmac-keyfile <path>` on the command line. The
   operator banner shows `(N bytes ok)`, `(MISSING)`, or `(BAD)` so
   you can confirm it loaded. The bytes never reach the UI.

4. **Rotator calibration.** One-time per controller:

   ```sh
   simple_sat_ops --control --calibrate-rotator --confirm-rotator-calibrate
   ```

   The antenna moves through three legs (park, az $90^\circ$, el
   $45^\circ$, park).
   Results write to
   `~/.local/share/simple_sat_ops/rotator_{az,el}_rate_dps`. The
   pursuit planner reads them at the next normal startup. See
   [Rotator calibration](#rotator-calibration---calibrate-rotator).

5. **T/R switch (optional).** If `/dev/ttyACM0` is present, the
   operator UI auto-probes it on start. No flags needed.

## Before you operate: licensing and authorization

Setting up the station is the easy part. Operating it carries legal
and institutional obligations, and they are not optional. Before you
run a pass - even as the person at the keyboard of a *remote* session -
every item below must apply to you. If any is missing, you are a
viewer, not an operator.

> **Safety.** This is the one chapter where "the software won't let me"
> is not the safeguard. The transmitter inhibit protects the hardware;
> nothing in the code checks your licence. That check is on you.

**Amateur-radio qualification (ISED Canada).** Under the University of
Calgary's rules - which follow from the authorization Global Affairs
Canada has granted - you personally need:

- a **Basic** ISED amateur-radio qualification to operate the antenna
  in *receive* mode, and
- an **Advanced** qualification to *transmit*, whether you are sitting
  at the RAO station or driving it remotely.

These qualifications are yours, not the station's. Hold the right one
before you take the corresponding role.

**Approval to operate the satellite.** Hearing the spacecraft is one
thing; commanding it is another. You must be approved by Dr. Burchill
before you operate FrontierSat itself.

**The rules of the road.** Be fluent in amateur-radio operating
practice and etiquette: band plans, station identification, listening
before you transmit, yielding to other users. The bands are shared and
the hobby runs on courtesy; operate accordingly.

**The RSSSA constraints.** FrontierSat is a remote-sensing space
system, so its operation is governed by Canada's **Remote Sensing
Space Systems Act (RSSSA)**, administered by Global Affairs Canada. The
Act, and the licence issued under it, constrain what the system may do
and how we may operate it. You are expected to know what the RSSSA is
and how it limits us, and to have read both our RSSSA application and
the GAC-issued licence before you operate.

**Your own account.** Operate from a *personal* account on the RAO
ground-station computer - never a shared or generic login. The
same-operator enforcement and the audit log both assume one human per
account, and the licensing obligations above are individual, so the
record has to be too.

Until the whole list is true for you, sit in viewer mode (no
`--control`), watch, and learn. That is exactly what it is for.

## A map of the cat: the tools

Everything from here to the appendices is the tool reference. It is
meant to be consulted, not held in your head: learn that these tools
exist and roughly what each is for, then look up the details when you
need them. This table is the outline you name the parts off.

One binary touches the hardware during a pass; the rest plan it,
review it, or take apart what it recorded.

| When you want to | Reach for |
|------------------|-----------|
| Run a live pass (the only tool that drives the radio and rotator) | [`simple_sat_ops`](#operator-ui-simple_sat_ops) |
| Find the next pass and plan a schedule | [`next_in_queue`](#pass-scheduling-next_in_queue) |
| Sanity-check a command list before you send it | [`agenda_check`](#agenda-review-agenda_check) |
| Pull frames out of a recorded capture offline | [Offline analysis tools](#offline-analysis-tools) |
| Bench bring-up, one-shot test transmits, IQ recording | [Bring-up and test tools](#bring-up-and-test-tools) |
| Confirm the math still holds after a change | [Unit tests](#unit-tests) |

## Operator UI: `simple_sat_ops`

The operator UI is an ncurses program that:

* propagates the satellite from a TLE (SGP4/SDP4) and shows live
  Doppler-corrected uplink and downlink frequencies,
* drives the SPID rotator on a worker thread,
* owns the B210 in RX continuously and switches to TX briefly during
  each uplink burst,
* broadcasts state to read-only viewers over a Unix-domain IPC
  socket,
* writes a continuous IQ and WAV capture into a per-pass folder
  under `/FrontierSat/Operations/<yyyymmdd>/<HHMMLT>/`,
* decodes AX100 frames on the fly and inserts each one into a shared
  SQLite packet database.

It also lets the operator compose and send individual uplink
telecommands interactively, or pull a list of timed telecommands
from disk and stream them across the pass at their scheduled
execution times.

### Modes: operator vs. viewer

```text
simple_sat_ops --control [<satellite>] [options]   # operator mode
simple_sat_ops [options]                            # viewer mode (an operator must be running)
```

**Operator (`--control`).** Opens the IPC socket
(`/run/sso/simple_sat_ops.sock`), takes ownership of the B210,
rotator, and pass folder. Refuses to start if another operator is
already running. The refusal message names the running operator and
points at the force-claim path (see
[Troubleshooting](#troubleshooting)).

**Viewer (no flag).** Connects to the running operator's socket and
renders the same state read-only. There is no longer a "track on my
own" standalone mode: without `--control`, if no operator is running,
the program exits with `operator not found: try simple_sat_ops
--control ...` rather than starting. A satellite name given without
`--control` is ignored - a viewer mirrors whatever the operator is
tracking. Press `c` then `y` inside the confirmation window to
force-claim. The running operator yields, the socket disappears, and
the viewer re-execs into `--control` with the same TLE and pass folder.

> **Drill (your first pass).** Find an operator already running, or
> start one yourself, then open a second `simple_sat_ops` with no
> flags. You are now a viewer: every panel updates and nothing you
> type reaches the hardware. Watch a full pass go by. Notice how the
> Doppler figures walk through the carrier as the satellite rises and
> sets, how the rotator panel trails the satellite az/el by a degree
> or two, and how the frame counter ticks when the link is good. When
> the pass ends you will have seen everything an operator does, at zero
> risk to anything.

### Command-line options

`--help` and `--help-full` are authoritative. Most-used flags:

| Flag | Purpose |
|------|---------|
| `--control` | Operator mode (one allowed at a time). |
| `--tle <path>` | Path to a 3-line TLE file. Default: `$HOME/.local/state/simple_sat_ops/active.tle`. Space form is tab-completable; `--tle=<path>` also works. |
| `<satellite_id>` | Name prefix to match in the TLE. Optional with `--control` (auto-discovered from the TLE name). Ignored without `--control` - a viewer mirrors the operator's target. |
| `--lat=<deg>` `--lon=<deg>` `--alt=<m>` | Observer override. Defaults to the Rothney Astrophysical Observatory (Priddis, SW of Calgary). |
| `--rotator-device <path>` | Override the SPID tty. |
| `--without-rotator` (alias `--without-hardware`) | Skip the SPID entirely. |
| `--without-tr-switch` | Skip the T/R switch probe. |
| `--tr-switch-device=<path>` | Override the T/R switch tty. |
| `--without-b210` | Run UI plus rotator only (skip the SDR). |
| `--sdr-type=uhd\|rtlsdr\|auto` | SDR backend (default `auto`: probe UHD, then RTL-SDR). See [SDR backends](#sdr-backends). |
| `--uhd-args=<args>` | UHD device args verbatim (e.g. `type=b200,serial=...`); overrides detection. |
| `--sdr-fpga=<path>` | Force a UHD FPGA image (a B2xx clone whose bitstream differs from stock). |
| `--sdr-device=<sel>` | RTL-SDR dongle index (for UHD prefer `--uhd-args`). |
| `--no-tx` | Open the SDR but block PA keying. The TX compose modal still shows preview and dry-run. |
| `--hmac-keyfile <path>` | Override the HMAC keyfile. |
| `--tc-file=<path>` | Telecommand list for the `A` auto-tcmd modal. |
| `--calibrate-rotator` `--confirm-rotator-calibrate` | One-shot calibration mode (see below). |
| `--without-rotator-pursuit` | Disable the pursuit / lead-aim planner; the track loop falls back to today's aim-where-sat-is-now logic. Useful for A/B on the bench. |
| `--scan-sky` `--scan-step=<deg>` | Drive the rotator through a sky grid, dwelling at each target. Bypasses the satellite-tracking gate. |
| `--always-record` | Start WAV and IQ capture immediately at open; don't gate on elevation. |
| `--live-waterfall` | Auto-launch the raylib `live_waterfall` viewer alongside the terminal UI. |
| `--self-test` | Print the resolved configuration and exit. Useful in scripts. |

`--help-full` also lists the TX safety gates and viewer options.

### SDR backends

`simple_sat_ops` talks to the radio hardware through a pluggable SDR
backend. By default (`--sdr-type=auto`) it probes the compiled-in
backends in order - UHD first, then RTL-SDR - and uses the first that
opens. Pick one explicitly with `--sdr-type=uhd` or `--sdr-type=rtlsdr`.
The active SDR and its transmit capability are shown on the RX panel
(`SDR <name>` or `SDR <name> (RX-only)`) and on the startup line.

**Confirm the hardware first with `sdr_probe`.** Before a pass, run
`sdr_probe` to see what's attached without starting `simple_sat_ops`. It
opens the UHD device the same way the operator UI would, prints its name
and the RX/TX channel counts and antennas, names the ports
`simple_sat_ops` uses (receive on `RX2`, transmit on `TX/RX`), and then
lists any RTL-SDR dongles. Use `sdr_probe --uhd-args=serial=...` to point
at a specific device when more than one is present.

**USRP (UHD) - the operational path.** The UHD backend transmits and
receives, so all operator functions are available.

If you run a **B210-ish clone whose FPGA differs from the stock image**,
that bitstream has to be loaded for the board to come up. A clone is
indistinguishable from a genuine board on the USB bus *except by its
serial number*, so the image is selected per-serial.

The bitstream is **not** in the repository (a large binary derived from
vendor IP, so it is git-ignored and never published). It lives on the
RAO ground-station computer under `/FrontierSat/sdr/`. One-time setup on
your machine:

1. Copy the image into your FrontierSat tree, at the same path used on
   RAO:

   ```
   mkdir -p /FrontierSat/sdr
   scp <user>@va6raogndstn:/FrontierSat/sdr/usrp_b210_fpga.bin /FrontierSat/sdr/
   ```

   If your FrontierSat root is elsewhere, use it - the tools resolve
   `$FRONTIERSAT_ROOT`, then `/FrontierSat`, then `~/FrontierSat`.

2. Find the device serial: run `sdr_probe`. It prints e.g.
   `USB serial: 30AA038`.

3. Map the serial to the image - add a line to
   `~/.local/share/simple_sat_ops/sdr_fpga_map`:

   ```
   30AA038 /FrontierSat/sdr/usrp_b210_fpga.bin
   ```

   (That file is created with a template, and your serial commented in,
   the first time `simple_sat_ops` opens the device.)

4. Confirm: re-run `sdr_probe`. It should report the clone image and the
   board should open (RX2 / TX-RX).

From then on the clone's image loads automatically whenever that serial
is seen; a genuine board with a different serial still gets the stock
image. To override the map for a one-off, pass `--sdr-fpga=<path>`, or
fold it into `--uhd-args="type=b200,serial=...,fpga=<path>"` (which wins
over everything). The serial is read via libusb; UHD's own
device-enumeration call segfaults on macOS, so it is not used.

**RTL-SDR - receive only.** An RTL-SDR dongle (`--sdr-type=rtlsdr`, or
auto with no USRP present) runs the full receive chain: tracking,
Doppler, recording, waterfall, and decode all work. It **cannot
transmit**: the TX compose (`t`) and auto-telecommand (`A`) modals open
for composing and preview but the allow-tx gate is forced off, and a
commit is refused with "TX not supported by this SDR (RX-only backend)".
Pick the dongle with `--sdr-device=<index>` if you have more than one.
The RTL backend is compiled in by default when `librtlsdr` is present
(it auto-disables if the library is missing; `-DWITH_RTL_SDR=OFF` forces
it off).

### Keyboard controls

Keyboard starts **unlocked**. Press `K` to toggle the lock; the status
line reads `unlocked` or `LOCKED`. Lock it when you want to lean on the
keyboard, or hand the screen to someone, without nudging the antenna.

| Key | Action |
|-----|--------|
| `K` | Toggle keyboard lock. |
| `T` | Start tracking the current satellite. |
| `s` | Stop tracking (and stop the rotator). |
| `r` | Stop tracking and return to (az=0, el=0). |
| `[` / `]` | Nudge antenna azimuth -5 / +5 deg. |
| `{` / `}` | Nudge antenna azimuth -1 / +1 deg (fine). |
| `,` / `.` | Nudge antenna elevation -5 / +5 deg. |
| `<` / `>` | Nudge antenna elevation -1 / +1 deg (fine). |
| `t` | Open the TX compose modal. |
| `A` | Open the auto-telecommand modal (requires `--tc-file`). |
| `:` | Enter the colon-command prompt. |
| `q` | Quit. |

The nudge keys stop satellite tracking before moving. They route
through the same worker thread as the main track loop, so they
don't block the UI.

Inside a modal (`t`, `A`, or the `:` prompt) every keystroke is
consumed by the modal until you commit or cancel. The jog
characters above are typed as plain text in that context.

### Status, RX, and TX panels

Screen layout (ncurses, redrawn at ~10 Hz):

* **Top status block.** Operator user, IPC role, mode banner,
  carrier frequencies (nominal and Doppler), TLE pinned for the
  pass, rotator panel (target az/el, current az/el, in-flight or
  stale flag), T/R switch panel, telemetry overlay.
* **Main pass region.** Satellite az/el, range, range-rate,
  sub-point lat/lon, altitude. Per-pass progress bar. Predicted
  pass parameters (AOS, LOS, max elevation, duration). Pass folder
  path.
* **RX panel** (when the B210 is open). Live IQ peak and RMS dBFS,
  frame counter from the live AX100 decode loop, shadow decoder
  frame counts, signal-quality estimate.
* **TX log panel** (bottom). Rolling N-line history of TX events:
  `draft>` as you compose, `sent>` once a command goes on the air,
  and a `notsent>` line *only* when a command did **not** reach the air,
  carrying the reason. A clean send leaves just the `sent>`
  record - the ground station does not acknowledge its own transmit,
  and the satellite's reply, if any, arrives on the downlink (the RX
  panel and packet database), not here. Viewers see the same lines
  via IPC.

Elapsed-time readouts (T/R switch last-TX, RX last-frame age) are shown
in a compact form that lists only the parts that matter: `2s ago`,
`1h 12s ago`, `3d 4h ago` - not a long fractional second count.

**Errors don't scramble the screen.** While the live screen is up, any
stderr from the radio/SDR code or a system library is redirected to
`sso_stderr.log` in the pass folder instead of landing on the panels
(the worst offender was a B210 USB-overflow message that could repeat
many times a second). Nothing is lost - it's captured for later - and
normal stderr is restored when the screen is torn down. On a clean quit
the program prints one final line: `No errors reported` if nothing was
written this run, or `Errors logged in <file>` pointing straight at it
(the check compares the log's size against its size at startup, so a
reused pass folder won't give a false positive from an earlier run).

### Colon-command prompt

Press `:` to open a one-line command bar at the bottom. The prompt
echoes keystrokes to viewers (debounced) so observers see what the
operator is about to commit. Enter fires, Esc cancels.

Line editing:

* **Up / Down** cycle a history of the commands you have run this
  session. The line you were typing is preserved and comes back when
  you press Down past the newest entry.
* **Tab** completes: the first word against the command names; any
  later word as a filesystem path.
* **`$VAR`, `${VAR}`, and a leading `~`** are expanded when a command
  reads a path. Tab completion keeps the `$VAR` / `~` prefix literal in
  the buffer and only completes the trailing component, so
  `:retarget $TLES/20260529/<Tab>` keeps `$TLES` and fills in the file.

Commands:

| Command | Effect |
|---------|--------|
| `:track` / `:stop` | Start / stop tracking the current satellite. |
| `:home` | Stop tracking and return to (az=0, el=0). |
| `:retarget <tle-file>` | Switch the tracked satellite mid-pass to the first one in the file (see below). |
| `:tx` / `:auto` | Open the TX compose / auto-telecommand modal. |
| `:freq <MHz>` | Override the nominal downlink frequency for the current pass (a value below 1e6 is read as MHz, otherwise Hz). |
| `:gain <dB>` | Set the AD9361 RX gain. |
| `:lo_offset <signed-kHz>` | Move the B210's LO offset to dodge a baseband artifact. |
| `:lo_bandwidth <kHz>` | Set the live waterfall's visible bandwidth (needs `--live-waterfall`). |
| `:spectrum <N>` | Render a spectrogram of the last `N` seconds of WAV/IQ via `gen_waterfall` (forked, non-blocking). |
| `:rs on\|off` | Reed-Solomon toggle (not yet runtime-wired; reports as much). |
| `:help` | List the commands. |
| `:quit` | Quit (`:q` and `:exit` too). |

The definitive set is whatever's wired up in `apps/main.c`'s
`cmd_dispatch`; the line-editing keys live in `cmd_handle_key`. Check
those if you need to be sure.

#### `:retarget <tle-file>`

Switch the satellite being tracked partway through a pass - for
example when a second satellite you also want is above the horizon at
the same time. The **first** satellite in the named file is used; its
name need not match anything already loaded. If you are tracking, the
antenna slews straight to the new target on the short path - it does
**not** unwind through (0, 0) first. A repeat `:retarget` on the same
file is a no-op; a different file swaps even when it names the same
satellite. The satellite display row and the viewer broadcast both
follow the new target. The path argument is expanded and
Tab-completable, so `:retarget $TLES/20260529/tle-20260529.tle` works.

### TX compose modal (`t`)

Multi-field modal for composing a single uplink burst. Fields:
payload (hex or ASCII), CSP v1 header (`prio`, `src`, `dst`,
`dport`, `sport`, `flags`), frequency override, gain, repeat count,
gap between repeats, preroll, and three safety-gate checkboxes
(`--allow-tx`, `--allow-high-power`, `--allow-hf-tx`).

Each field edit is debounced (~200 ms) and broadcast to viewers as
a `tx-preview` event, so observers see the draft before commit.
Enter sends a `tx-request`. The burst runs on its own thread, which
brings the transmit chain up (on the B210's `TX/RX` port), sends, and
powers the transmit chain back down; **receive keeps running
continuously the whole time** (on `RX2`), so the satellite's reply in
the seconds right after a command isn't missed. A `tx-command-sent`
event is recorded in the TX log. If the burst never reaches the air -
no SDR, frame-build failure, UHD error, or `--tx-dry-run` - a
`tx-not-sent` event carries the reason instead. Esc cancels.

On an [RX-only SDR](#sdr-backends) (RTL-SDR), the modal still opens for
composing and preview, but the allow-tx gate is forced off and a commit
is refused with "TX not supported by this SDR (RX-only backend)".

### Auto-telecommand modal (`A`)

Requires `--tc-file=<path>` on the command line. The file format
matches what the wider CalgaryToSpace tooling uses: one telecommand
per line. Blank lines and whole-line `#` comments are ignored, and an
inline trailing comment is stripped from a command:

```
CTS1+function_name(arg1,arg2)@tssent=<unix_ms>@tsexec=<unix_ms>@resp_fname=<f>!
# a whole-line comment
CTS1+adcs_identification()@tsexec=<unix_ms>!   # an inline comment, ignored
```

A telecommand always ends with `!`, so a `#` is treated as a comment
only when whitespace precedes it; a `#` inside the command text (no
space before it) is left intact, so a command is never silently
truncated on its way to the air.

The `A` modal lists the commands, lets the operator set per-pass
parameters (power, repeats, delay), and ticks the same three TX
safety gates. Once running, each tick checks whether the next
command's `@tsexec=` has arrived. If it has, the command is staged
into the same `tx-request` slot the compose modal uses. The TX
dispatch is async (submit + poll), so the rotator, redraw, IPC, and
the next auto-tcmd tick keep running while each burst is in flight.

The auto-tcmd loop respects the safety gates and the LOS guard.
Once the pass ends, it stops sending commands. On an
[RX-only SDR](#sdr-backends) the loop refuses to send (the same RX-only
gate as the compose modal).

### Finding telecommands and their arguments

`simple_sat_ops` only carries telecommands to the satellite - it does
not define them. The authoritative documentation lives with the flight
firmware in the CalgaryToSpace repository:

- Mission operations ICD (start here - the directory of operations
  documents):
  <https://github.com/CalgaryToSpace/CTS-SAT-1-OBC-Firmware/tree/main/docs/Mission_Operations>
- The command list itself - commands, their arguments, and the
  responses to expect:
  <https://github.com/CalgaryToSpace/CTS-SAT-1-OBC-Firmware/blob/main/docs/Mission_Operations/Telecommands_and_Config_Variables.md>

Use the command list to look up the exact function name, its argument
order and types, and the form of the reply on the downlink. The command
you type into the `t` compose modal (or list in a `--tc-file`) is the
same `CTS1+function_name(...)!` string documented there.

> Note: the manual link points at `main`. The image you are actually
> flying may be a tagged release rather than `main`; when in doubt,
> read the docs at the firmware tag that is on the spacecraft.

#### Populating a `--tc-file` for auto-commanding

*(To be written.)* This subsection will cover turning the documented
commands into a `--tc-file` for the `A` auto-telecommand modal: the
per-line `CTS1+...@tssent=...@tsexec=...!` format, how to set the
execution times, and how to dry-run the list with
[`agenda_check`](#agenda-review-agenda_check) before a pass.

### Rotator calibration (`--calibrate-rotator`)

The pursuit planner needs the SPID's deg/s slew rates on each axis.
They're set on the SPID controller's front panel and aren't reported
via the W-frame STATUS reply, so we measure them by moving the
antenna across a known arc and watching the encoder.

This is the SPID quirk promised in the foreword. The controller
updates its reported position the instant it receives a SET, so the
first STATUS after a move is a confident lie: the target dressed up as
the truth. The calibrator works around it by discarding the first
couple of samples and waiting until the encoder is genuinely crawling
toward the target before it trusts a number.

```sh
simple_sat_ops --control --calibrate-rotator --confirm-rotator-calibrate
```

Three legs:

1. Park to (az=0, el=10). The starting move is not timed; it just
   establishes a known reference.
2. Sweep az 0 to 90 deg, streaming STATUS samples.
3. Sweep el 10 to 55 deg, streaming STATUS samples.

For each timed leg the calibrator:

* drops the first two samples after the SET (the SPID firmware's
  position counter updates atomically on receipt, so the first
  STATUS reflects the **target**, not the encoder),
* drops the last two samples (deceleration and settle),
* finds the first and last interior samples that show motion
  ($>0.1^\circ$ from their neighbor),
* computes the rate as $|\Delta v| / \Delta t$ across that span.

If the firmware actually target-latches and never reports encoder
progression, the interior shows no motion and the calibrator bails
with `bad_rate` instead of producing a misleading number.

On success it writes
`~/.local/share/simple_sat_ops/rotator_{az,el}_rate_dps` (plain
text, one float each). Re-run any time the controller's rate
settings change.

**Safety.** `--confirm-rotator-calibrate` is required because the
antenna physically moves through all three legs without further
prompting. Clear the mast area before you run it. A stray
`--calibrate-rotator` left in a script can't move anything without the
confirm flag.

### Pursuit tracking

The rotator slews at a fixed, modest rate. The satellite, near the
apex of a high pass, does not: it can cross the sky faster than any
reasonable antenna can follow. Aiming at where the satellite is *right
now* means arriving late on every tick, worst at the top of the pass
where the signal is best. Pursuit aims instead at where the satellite
is about to be, planning the whole arc in advance so the rotator's
steady slew coasts through the curve rather than chasing it.

When rotator calibration is on disk and `--without-rotator-pursuit`
is **not** passed, then at pass start (`T`, or auto AOS) the
operator UI:

1. Pre-samples the satellite's mech-frame az/el every 1 s across
   the pass on a working copy of the prediction (so the live
   displayed az/el is not disturbed), applies the flip mech-coord
   mapping for high-elevation passes, and unwraps azimuth across
   0/360 in time order.
2. Asks the pursuit planner (`src/orbit/pursuit.c`) for a
   rate-feasible whole-pass antenna trajectory.
3. On each track-loop tick reads the next waypoint via
   `pursuit_aim_at(jul_utc)` and submits it through the same path
   today's aim-where-sat-is-now logic uses. The constant-rate slew
   between waypoints interpolates the segment.

stderr at pass start looks like:

```
pursuit: plan built 64 waypoints, max_err=1.42 mean_err=0.61 deg, 3 iter
```

If the planner produces a plan with max error > 30 deg the plan is
discarded with `pursuit: plan max_err=N.N deg > 30; disabled` and
the track loop falls back to the old aim-now behavior. Same
fallback fires when calibration is absent, the planner fails to
build, or `--without-rotator-pursuit` is set.

The plan is freed at LOS, on any mid-pass abort (out-of-bounds az,
operator presses `s`), and at process exit. See
[Pursuit planner internals](#pursuit-planner-internals).

## Pass scheduling: `next_in_queue`

A read-only CLI that reports upcoming passes over the observer
location.

```text
next_in_queue <satellite_name> [options]
next_in_queue <min_alt_km> <max_alt_km> [<satellite_name>] [options]
next_in_queue --tle <path> [<satellite_name>] [options]
next_in_queue --trajectory-id=<id> [options]
```

**Named form (the common case).** Give just a satellite name and
`next_in_queue` finds its TLE for you and reports the next week of
passes - no altitude limits needed, because you already know the
object. It looks, in order:

1. the newest dated TLE under `/<satellite_name>/TLEs` (the
   `<date>/tle-<date>.tle` convention - e.g. `FrontierSat` resolves
   to `/FrontierSat/TLEs/20260529/tle-20260529.tle`, picking the
   latest date);
2. failing that, the default `$HOME/.local/state/simple_sat_ops/active.tle`,
   filtered to the named object;
3. failing both, it errors. Override the whole search with `--tle=<path>`.

**Catalog-scan form.** Give an altitude band (`<min_alt_km>
<max_alt_km>`) to scan every object in the default TLE, optionally
narrowed by a trailing name prefix.

Common options (see `--help-full` for the full list):

| Flag | Purpose |
|------|---------|
| `--tle <path>` (also `--tle=<path>`) | TLE file. Default: `$HOME/.local/state/simple_sat_ops/active.tle`. |
| `<satellite_name>` | Literal case-sensitive name prefix. Bypasses the constellation filter, extends the window to a week. |
| `--regex=<pattern>` `--ignore-case` | Regex match on the satellite name. |
| `--list` | Print every matching pass (default is just the next one). |
| `--reverse` | Sort latest-first. |
| `--max-passes=<n>` | Limit the output. |
| `--max-minutes=<n>` | Window. Default 1440 (one day); satellites named via positional get a week. |
| `--min-elevation=<deg>` `--max-elevation=<deg>` | Peak-elevation filter. |
| `--min-altitude-km=<km>` `--max-altitude-km=<km>` | Orbital-altitude filter. |
| `--minutes-offset=<n>` `--t0=<yyyy-mm-ddThh:mm:ss>` | Advance `now` by `n` minutes, or pin an absolute UTC start. |
| `--all-satellites` | Include Starlink, OneWeb, and other commercial constellations (default off). |
| `--show-radio-info` | Annotate matches with amateur-radio info from `active_radios.txt` (looked up alongside the TLE). |
| `--lat= --lon= --alt=` | Observer override. |
| `--trajectory-id=<id>` | Use a propagated SSM trajectory instead of a TLE. |

Examples:

```sh
# Next week of FrontierSat passes - finds /FrontierSat/TLEs/<date>/tle-<date>.tle
next_in_queue FrontierSat --list

# Same, but force a specific TLE file instead of auto-discovery
next_in_queue FrontierSat --list --tle $TLES/20260529/tle-20260529.tle

# Soonest visible pass above 10 deg, in the next 2 hours (catalog scan)
next_in_queue 0 2000 --min-elevation=10 --max-minutes=120

# All passes for the next 3 hours above 30 deg (catalog scan)
next_in_queue 0 2000 --list --max-minutes=180 --min-elevation=30

# ISS-class passes with amateur info annotation.
# With --tle the altitude positionals are not allowed; use the
# --min/--max-altitude-km flags instead.
next_in_queue --tle $TLES/amateur.tle --list \
              --min-altitude-km=300 --max-altitude-km=500 \
              --regex='ISS|ZARYA' --ignore-case --show-radio-info
```

`--tle` accepts the space form (TAB-completable) and the legacy
`=`-form interchangeably. The help text shows the space form.

## Agenda review: `agenda_check`

Telecommand-list reviewer. Reads a TC file (same format the
operator UI consumes via `--tc-file`) and prints it back with the
embedded `@tssent=` and `@tsexec=` unix-millisecond timestamps
converted to ISO 8601, so the operator can confirm the schedule
before keying anything on air.

```text
agenda_check [--local-time] [--no-dup-check] [--prune-dups] [--tle <file>] [<file>]
```

| Flag | Effect |
|------|--------|
| `--local-time` | Render times in the host's local timezone (default UTC). |
| `--no-dup-check` | Skip the duplicate-line audit (substitute the timestamps only). |
| `--prune-dups` | Drop verbatim-duplicate command lines (keep the first occurrence). Prints a count of how many were pruned. |
| `--tle <file>` | (sgp4sdp4 builds only) Propagate the first satellite in `<file>` and prepend the execution date-time plus sub-satellite lat/lon/alt to each command. Leaves the command intact (raw unix-ms preserved). |

Verbatim duplicates are flagged inline with a red `DUP(line N)>`
prefix pointing at the line number of the first occurrence. Pipe
to `grep` and the plain prefix survives.

With `--tle`, each command line is prepended with `<iso-time>
lat=<deg> lon=<deg> alt=<km>` (in mech coords, lat one decimal,
fixed-width). Without `--tle`, the inline `@tssent=` and `@tsexec=`
timestamps are humanized in place.

Comments follow the same rule the operator UI uses (the rule lives in
one shared place, so the audit and the transmit path can't disagree).
Whole-line `#` comments are passed through verbatim. An inline trailing
comment (whitespace + `#...`) is split off the command: it is preserved
in the output, but excluded from the command count and the
duplicate/timed keys - so the same command carrying two different
comments is still flagged as a duplicate, and a `#` inside a command is
left intact.

After processing, a stderr summary lists:

```text
agenda_check: 918 commands total, 780 non-duplicate, 6 unique timed
```

Where:

* *total* = every command line (non-comment, non-blank).
* *non-duplicate* = distinct verbatim lines. Same command at
  different times stays distinct.
* *unique timed* = distinct command identities (with `@tssent=` and
  `@tsexec=` values masked out) among lines that carry a timestamp.
  A `fs_list_directory_json(/,0,20)` scheduled 30 times across the
  day counts once.

`--prune-dups` returns 0 with `agenda_check: pruned N duplicate
line(s)` on stderr. The default flag mode returns 3 when duplicates
were present (so CI can gate on it).

## Offline analysis tools

### `gen_waterfall`

Renders a SatNOGS-style waterfall PNG from a raw int16 IQ file. No
libpng or libz dependency: DEFLATE is stored-only and the FFT is
inlined. The operator UI invokes it via `:spectrum N` and at end of
pass when an IQ sidecar exists. You can also run it directly:

```sh
gen_waterfall <iq-path> <sample-rate-hz> [<out-png>] [options]
gen_waterfall <audio.ogg> [<out-png>] [options]
```

The PNG path is an optional positional (not a flag). Useful options:
`--fft=<n>`, `--db-min=<dB>`, `--db-max=<dB>`, `--zoom-khz=<kHz>`,
`--dc-notch`, `--detrend=<mode>`, `--pdf=<path>`. It renders the **full
capture width by default**; `--zoom-khz=<kHz>` narrows the view. The
output uses a median-subtracted noise floor and a viridis palette.

A **SatNOGS `.ogg`** is accepted directly. It is FM-demodulated audio,
not IQ, and carries its own sample rate, so there is **no rate
positional** - the PNG is positional arg 2. It's rendered as an audio
spectrogram (the real signal is fed as IQ with Q=0, so the spectrum
mirrors about DC and the FSK tones appear at +/-f). Needs libsndfile.

### `rx_replay`

Plays a previously-captured `.iq` (or `.wav` FM-demoded) sidecar
through the same RX session and decode pipeline `simple_sat_ops`
uses live. Lets you re-run the decoder with different settings
(FIR, squelch, NCO) without taking a fresh capture.

```sh
rx_replay <iq-or-wav-path> [--rate=<Hz>] [--lo-shift-khz=<N>] [--viterbi] [--tle=<path>] [...]
```

(All options are `=`-form. There is no `--gain`; shift the loaded IQ
with `--lo-shift-khz=` and set the sample rate with `--rate=`.)

The input format is picked from the extension: `.iq` is headerless
int16 I/Q (the two-pass IQ decoder); `.raw` is headerless S16_LE PCM;
anything else is read as a `.wav`. A **SatNOGS `.ogg`** recording is
also accepted directly: it is the receiver's FM-demodulated *audio*
(the discriminated voltages, just Vorbis-compressed), so `rx_replay`
decodes it to PCM in memory via libsndfile and runs the **FM-audio
chain** - the same path as a `.wav`, not the IQ path. Requires the
build to have libsndfile; otherwise an `.ogg` errors with a hint.

```sh
rx_replay /FrontierSat/SatNOGS/.../satnogs_<id>_<utc>.ogg
```

See [Replaying SatNOGS recordings](#replaying-satnogs-recordings).

With `WITH_SGP4SDP4` and a TLE, `rx_replay` can also re-derive
Doppler from the recorded sidecar timestamps. The TLE-based
predictions in some older `rx_replay` versions show inflated range
numbers; for new TLE / SGP4 code, model on `apps/next_in_queue.c`
and `src/orbit/prediction.c` (the known-good path).

### `decode_inspector`

Interactive raylib-backed viewer for offline IQ. Scrub through a
capture, draw boxes around bursts, watch the
decoder run stage by stage - ASM, Golay(24,12), descrambler,
Reed-Solomon, CSP - and tune the front end with the same flags
`gen_waterfall` accepts (`--zoom-khz=`, `--detrend=`, `--dc-notch`,
`--db-min=`/`--db-max=`, `--lo-shift-khz=`, ...). A `--live` mode tails
an in-flight capture. Optional; built only when raylib is installed.

Boxes you draw are written to `<iq>.boxes.csv` and, in an
`rx_replay`-compatible form, `<iq>.box_anchors.csv` - feed the latter
back with `rx_replay --anchor-csv=<iq>.box_anchors.csv` to re-decode
exactly the slice you marked.

### Replaying SatNOGS recordings

SatNOGS publishes a pass as an `.ogg` audio file - the ground station's
FM-demodulated baseband, i.e. the discriminated voltages, Vorbis-
compressed. Because it is already demodulated audio (not IQ), you run
it through the **FM-audio** path, the same one a `.wav` uses:

```sh
rx_replay /FrontierSat/SatNOGS/<date>/satnogs_<id>_<utc>.ogg
```

`rx_replay` detects the `.ogg`, decodes it to PCM in memory via
libsndfile, reads the sample rate from the file (SatNOGS audio is
typically 48 kHz), and runs the decoder. Add the usual flags as needed
(`--bit-rate=`, `--window-s=`, `--tle=`, `--satellite=`, ...). Two notes:

- It is **audio, not IQ** - do not pass `--iq`, and the IQ-only knobs
  (`--lo-shift-khz=`) don't apply. The discriminated audio is what the
  decoder's FM-audio chain expects.
- A clean decode still depends on the recording: a weak or off-tune
  SatNOGS pass may yield few or no frames even though the file replays
  fine.

For a visual, render the `.ogg` as an audio spectrogram (no rate arg):

```sh
gen_waterfall /FrontierSat/SatNOGS/<date>/satnogs_<id>_<utc>.ogg wf.png
```

(`decode_inspector` is IQ-native and does not yet take `.ogg` audio.)

### `beacon_detect`

Cadence-matched beacon detector for the FM-demod baseband WAV that
every pass produces. Prints candidate timestamps. Useful for
triaging "did we hear anything?" on a noisy pass without spinning
up the full decoder.

```sh
beacon_detect <wav-path> [--threshold-sigma=<k>] [--lattice-period-s=<s>] [--min-spacing-s=<s>] [--csv=<path>]
```

### `fm_preview`

Builds an audible FM-modulated WAV from a baseband WAV. Useful for
producing a "what you'd hear on a handheld" preview from a recorded
pass. Drop the output into VLC or any media player.

```sh
fm_preview --in=<baseband.wav> --out=<output.wav> [--deviation=<Hz>] [--carrier=<Hz>] [--time-stretch=<x>]
```

The input and output are `--in=`/`--out=` flags, not positional
arguments.

### `packet_query` and `packet_browser`

`packet_query` is a CLI grep over the shared SQLite packet database
(the shared `$FRONTIERSAT_ROOT/packet_db.sqlite`, falling back to
`$HOME/.local/share/simple_sat_ops/packets.db`; override with
`$SSO_PACKET_DB` or `--db=<path>`). Filter by satellite, frame type,
time range, source tool, or capture origin. Output as a table
(default), JSON, CSV, or raw bytes via `--format=table|json|csv|raw`.
The `json`, `csv`, and `raw` forms emit the **whole** payload and
decoded summary - there is no length cap, so a large packet comes out in
full. `table` is the one-line-per-frame summary view; use the other
three when you want every byte.

`packet_browser` is an ncurses TUI over the same database. Scroll
through frames, expand to inspect the raw bytes and decoded fields,
re-run the framer with different options. The detail pane shows the
full payload as a multi-line hex dump (with a `... N more bytes` note,
pointing at `packet_query --format=raw`, if it overflows the visible
rows) and wraps each decoded-body line across as many rows as it needs,
so a long `tcmd_response` is readable rather than clipped at the right
edge.

A telecommand response is text ended by a zero byte; the decoded display
stops at that end marker, so trailing framing/parity bytes don't show up
as a garbage tail after the message. The raw byte dump still shows
everything.

```sh
packet_query --satellite=FrontierSat --since=1h --format=json
packet_browser
```

## Bring-up and test tools

The ALSA-based TX helpers that drove the legacy transceiver
(`tx_tone`, `tx_white_noise`, `tx_frame`) now live in
[Appendix A](#appendix-a-what-didnt-stick---the-ft-991a-and-ic-9700-radio-path)
with the rest of the radio path.

### `tx_frame_sdr`

Offline IQ rendering tool for an uplink burst. Builds the CSP and
AX100 frame from a payload and modulation parameters, then dumps
the resulting IQ via `--dump-iq=<path>` (or prints sizes via
`--dry-run`). The streamer path is gone. Live RF goes through
`simple_sat_ops`'s in-process TX burst; `tx_frame_sdr` is for bench
inspection and offline tooling.

### `b210_rx_capture` and `b210_gain_sweep`

Standalone RX capture (`b210_rx_capture`) for cases where you want
to record without spinning up the operator UI. Won't share the
B210 with a running `simple_sat_ops`.

`b210_gain_sweep` steps the AD9361 RX gain across the configured
range and logs the resulting peak and RMS levels. Useful for
finding the linear regime and for verifying that the 51 Hz comb
(impulsive spikes at mid gain settings, see
`src/hw/b210_rx_tx_core.h`) is or isn't present given the AD9361
tracking-loop settings.

### `live_waterfall`

A raylib viewer that tails the operator UI's live `.iq` and renders
a scrolling viridis spectrogram in a tall narrow window. Spawn it
automatically with `simple_sat_ops --live-waterfall`, or run it
standalone against any in-flight `.iq` file. Closing its window
leaves the recording running.

### `uplink_test`

End-to-end uplink test: builds a frame with a known HMAC,
modulates it, decodes it, asserts the round-trip is byte-clean.
Catches HMAC, AX100 framing, and modem regressions. Runs in CI on
every push.

### `rx_decode`

Library-style decoder driven from the command line. Reads a binary
frame blob, runs it through ASM, Golay, descrambler, Reed-Solomon,
and HMAC verification, and prints what each stage saw. Useful for
post-hoc inspection of bytes captured outside `simple_sat_ops`'s
live path.

### `lifetime`

Toy orbit-decay estimator. Reads a TLE, extrapolates a B*-derived
decay rate, prints an estimated time on orbit. The repository
README flags this as **inaccurate**. Treat the number as
ballpark-only and don't make scheduling decisions from it.

## Unit tests

Each `*_selftest.c` binary emits TAP (Test Anything Protocol). Run
manually:

```sh
build/rs_selftest
build/antenna_rotator_selftest
build/tle_csv_selftest
build/modem_iq_selftest
build/modem_fsk_selftest
build/sw_nco_selftest
build/beacon_cts1_selftest
build/csp_selftest
build/biquad_selftest
build/fir_decim_selftest
build/monitor_squelch_selftest
build/iq_burst_selftest
build/ax100_selftest
build/tx_burst_selftest
build/packet_db_selftest        # needs SQLite
build/prediction_selftest       # needs sgp4sdp4
build/pursuit_selftest          # no extra deps
build/unit_test_runner          # optional ncurses aggregator
```

`unit_test_runner` discovers every `*_selftest` binary under
`build/` and renders a collapsible group view of the TAP output.
Skipped if ncurses is absent.

The lint script `scripts/lint_warnings.sh` runs the whole build
under gcc-15 with `-Wformat-truncation=2 -Werror=format-truncation`
to catch format-truncation bugs Apple clang misses. Run it on
every nontrivial source change:

```sh
bash scripts/lint_warnings.sh           # full reconfigure + build
bash scripts/lint_warnings.sh --quick   # reuse the build-lint cache
```

## Architecture notes

### IPC: one operator at a time

A satellite admits one hand at a time. Two operators keying the same
rotator and radio would be a brief and expensive comedy, so the
software elects a single operator and makes everyone else a spectator.

The operator UI binds a Unix-domain socket at
`/run/sso/simple_sat_ops.sock` and writes its PID to
`/run/sso/simple_sat_ops.pid`. Viewers (and helpers like
`tx_frame_sdr`'s operator mode) connect, send a `hello`, and read
back a `welcome` that carries the operator's Unix user and pass
folder. Helpers verify the user matches their own `$USER` before
proceeding. This stops `bob`'s `tx_frame_sdr` from accidentally
posting events to `alice`'s operator session.

`simple_sat_ops --control` refuses to start when another operator
is already running. It probes first (as a transient viewer
connection), then prints the live operator's user and PID and
points at the force-claim path:

```
simple_sat_ops: --control refused: operator already running as user=alice pid=12345.
  To take over, run a viewer (no --control) and press
  'c' then 'y' to force-claim; the running operator
  will yield and your viewer will re-exec into --control.
```

The force-claim sends `SIGUSR1` to the running operator (which
treats it as a clean-exit signal), waits up to 5 s for the socket
to disappear, then `execv`s itself into `--control` with the same
TLE and pass folder. No hardware race.

If the bind fails with no operator detected by the probe, the most
likely cause is a stale socket and pid file from a crashed previous
operator. The error message tells you to remove
`/run/sso/simple_sat_ops.{sock,pid}` and retry.

### Worker threads

The main loop is cooperative single-threaded for the ncurses
surface but pushes three blocking-I/O domains onto dedicated
pthreads:

* **RX session worker** (`src/pipeline/rx_session.c`). Owns the
  B210 / UHD pump and the WAV / IQ writers, and receives continuously
  for the whole pass. The main loop submits TX bursts asynchronously via
  `rx_session_submit_burst` and `rx_session_poll_burst`; each burst runs
  on its own thread and brings the transmit chain up and back down
  *without* stopping, retuning, or reclocking receive - so the USB link
  keeps draining and the radio's buffer can't overflow during a burst
  (an earlier stop-the-world transmit flooded the log right when the
  reply was due). Powering the transmit chain down between bursts still
  guarantees nothing is left on the air while receiving.

* **Rotator worker** (`src/hw/antenna_rotator_async.c`). Polls the
  SPID STATUS at 2 Hz and drains a latest-wins SET slot. The UI
  reads a snapshot non-blockingly. A 500 ms VTIME hang from an
  unplugged cable no longer freezes the operator panel.

* **Audit-log writer** (`src/ipc/sso_audit.c`). A 256-slot ring
  buffer drained by a writer thread that opens, flocks, writes, and
  closes `runs.log` per event. The producer side
  (`sso_audit_event`) is non-blocking and stays sub-microsecond per
  call.

The spectrum-render path (`:spectrum N`) is also threaded: an
on-demand worker forks `gen_waterfall` and waits for completion
without blocking the UI.

### TX safety gates

All TX-capable tools default to TX-inhibited so refactors and
bring-up runs can't accidentally key the PA. Three gates, all
opt-in:

| Gate | Required when |
|------|----------------|
| `--allow-tx` | Any PTT-on or burst-commit. Without it the tool configures the radio (frequency, mode, MOD source, power) but stops at PTT with a clear stderr message. |
| `--allow-high-power` | `--tx-power` above 10%. |
| `--allow-hf-tx` | PTT below 100 MHz (HF). |

PTT-off is always passed through, even with the inhibit set;
releasing TX is never blocked.

In `simple_sat_ops` the same three gates appear as checkboxes on
the TX compose modal and the auto-tcmd modal. The values are
broadcast to viewers in the `tx-preview` event so a watching
operator can see them before commit.

### Pursuit planner internals

`src/orbit/pursuit.c` is the pure-function planner. Inputs:

* pass window `[jul_aos, jul_los]`,
* rotator slew rates `r_az_dps`, `r_el_dps`,
* antenna bounds (`ANTENNA_ROTATOR_MINIMUM_*`, `MAXIMUM_*`),
* antenna's starting position `(a0_unwrapped, e0)`,
* a `pursuit_sat_sample_fn_t` callback that returns the satellite's
  unwrapped mech-frame `(az, el)` at an arbitrary Julian date.

The planner discretises the pass into waypoints at
`waypoint_dt_s = 5 s` and the cost grid at `dense_dt_s = 1 s`.
Initial waypoints seed at the satellite's predicted positions; the
first waypoint is pinned to `(a0, e0)`. Iteration is alternating
forward and backward sweeps. For each interior waypoint the
planner projects the unconstrained optimum (`sat_pos(T_k)`) into
the intersection of the two rate-feasible boxes implied by its
neighbors. When the intersection is empty (the sat is moving
faster than the rotator can keep up), it falls back to the
forward-feasible box only and aims its boundary closest to the next
waypoint. Iteration stops once the cost stops improving by
`cost_improvement_eps = 0.5 %` or after `max_iter = 6` (clamped at
16).

The result is a rate-feasible trajectory whose waypoints are
already in unwrapped mech coords. `pursuit_aim_at(plan, jul_utc)`
returns the *next* waypoint after `jul_utc`. That's where the
operator UI tells the rotator to go, and the constant-rate slew
between waypoints interpolates the segment for free.

The selftest at `unit_tests/pursuit_selftest.c` covers 38 cases:
linear within rate, linear above rate, an apex pass (which beats a
naive trail-the-sat baseline), az wrap, boundary clamps,
degenerate inputs (NULL ptrs, empty window, zero rate), the
iteration cap, plan-lookup window, and the rate-file I/O
round-trip (with HOME redirected to a tempdir so the user's saved
values are not disturbed).

## File layout

| Path | What's there |
|------|---------------|
| `/FrontierSat/` | Top-level data root. Override with `$FRONTIERSAT_ROOT`; falls back to `~/FrontierSat` on dev hosts that lack `/FrontierSat`. |
| `/<satname>/TLEs/` (e.g. `/FrontierSat/TLEs/`, `$TLES`) | Dated TLE files, conventionally `<date>/tle-<date>.tle`. `simple_sat_ops --control` with no `<satellite_id>`, and `next_in_queue <satellite_name>`, both load the newest dated `*.tle` found here (searched recursively). Note: an explicit `--tle <path>` is **not** resolved under this directory - it is taken relative to the working directory (a CSV TLE is auto-converted to a temp `.tle`). |
| `/FrontierSat/Operations/<yyyymmdd>/<HHMMLT>/` | Per-pass folder. Holds the pass's WAV, IQ, decoded frames, TLE snapshot, doppler / lo_offset / burst sidecars, and the end-of-pass spectrogram and waterfall PNG. |
| `/FrontierSat/Operations/current` | Symlink the operator UI keeps pointing at the most recent pass folder. |
| `/FrontierSat/captures/` | One-off `b210_rx_capture` outputs. |
| `/FrontierSat/Testing/` | Bench captures and analysis. |
| `~/.local/state/simple_sat_ops/active.tle` | Default TLE file (the `--tle` default). |
| `~/.local/share/simple_sat_ops/rotator_az_rate_dps` | Calibrated rotator azimuth slew rate (deg/s). |
| `~/.local/share/simple_sat_ops/rotator_el_rate_dps` | Calibrated rotator elevation slew rate (deg/s). |
| `~/.local/share/simple_sat_ops/carrier-trim-hz` | Per-host carrier-trim offset that lands the B210's analog LO on the requested frequency. |
| `~/.local/share/simple_sat_ops/packets.db` | Shared SQLite packet database. |
| `~/.local/share/simple_sat_ops/runs.log` | Audit log (dev-host fallback when `/var/log/sso/` isn't writable). |
| `/run/sso/simple_sat_ops.sock` | Operator IPC socket. |
| `/run/sso/simple_sat_ops.pid` | Operator PID. |
| `/var/log/sso/runs.log` | Audit log (production path). |

## Troubleshooting

Before guessing, three tools narrow almost anything down:

- **`simple_sat_ops --self-test`** prints the fully resolved
  configuration and exits without touching hardware: the active mode,
  the SDR backend and TLE, the HMAC keyfile path *and whether it
  loaded*, Doppler/frequency settings, and the rotator/observer config.
  A missing file or a flag that didn't take shows up here - including
  the exact path a keyfile or TLE was expected at.
- **`sdr_probe`** (see [SDR backends](#sdr-backends)) confirms which SDR
  is attached, which FPGA image will load, and which ports are RX vs TX.
- **The TX log panel** (and the `tx.log` in the pass folder) record one
  line per composed command with the outcome, and - when a burst didn't
  reach the air - the reason. Most TX problems are answered right there.

The audit log and the stderr printed at startup are the next places to
look. Specific symptoms:

**`simple_sat_ops: --control refused: operator already running as user=alice pid=12345`**

Another operator is already there. Confirm with `ps -p 12345 -o
user,pid,cmd`. If you legitimately need to take over, run a viewer
(`simple_sat_ops` with no `--control`) and press `c` then `y`.

**`simple_sat_ops: --control: socket bind failed. If this is from a crashed previous operator, remove /run/sso/simple_sat_ops.{sock,pid} and retry.`**

The probe saw no operator but the bind still failed. Almost always
a stale socket and pid file from a crashed previous operator.
Confirm with `ls -la /run/sso/simple_sat_ops.*`, then `rm` both
files and retry.

**`pursuit: no calibration on disk; run \`simple_sat_ops --calibrate-rotator --confirm-rotator-calibrate\` to enable lead-aim`**

The pursuit planner needs rotator slew rates to plan a pass
trajectory. Without them, tracking falls back to today's
aim-where-sat-is-now logic. That works fine; it's just slightly
less accurate at apex on high-elevation passes. Run calibration
once to enable the planner.

**`pursuit: plan max_err=N.N deg > 30; disabled`**

The planner produced a plan whose worst-case pointing error exceeds
$30^\circ$. Most often this means the calibrated rates are way off (the
SPID can't actually keep up with the predicted angular velocity).
Re-run `--calibrate-rotator`. Tracking falls back to the aim-now
logic for this pass.

**`Warning: could not read SPID position; check that the Rot2ProG is in 'A' mode`**

The async rotator worker didn't see an OK STATUS reply in the
1500 ms startup window. Check the SPID front panel; it should read
something like `A 0000 0000` (mode A = remote control).
Power-cycle the SPID if it's stuck. With the cable unplugged or
the controller off, the operator UI still starts; it just renders
`?` on the rotator panel and the track loop won't drive the
antenna.

**`(BAD)` or `(MISSING)` on the HMAC banner**

Uplink bursts won't make it into the satellite. Check the keyfile
exists and has the expected length:

```sh
ls -la /FrontierSat/HMAC/frontiersat_hmac
ls -la "$HOME/.local/state/simple_sat_ops/frontiersat_hmac"
wc -c "$HOME/.local/state/simple_sat_ops/frontiersat_hmac"
```

Or pass `--hmac-keyfile <path>` to override.

**TX-inhibit messages**

The TX compose modal won't commit until the `allow-tx` checkbox is
ticked (and `allow-high-power` if `--tx-power > 10%`, and
`allow-hf-tx` below 100 MHz). The auto-tcmd modal works the same
way. Toggle them once per pass.

**TX composed but nothing transmits (no TRX LED, no RF)**

The PA only keys for the brief moment a burst is on the air, and a
burst that's refused never keys at all - so this is almost always a
refused burst, not a hardware fault. Read the TX log line for that
command (or `tx.log`): `sent>` means it went out (the LED flash is just
short - raise the repeat count to see it); `notsent>` / `err>` carry the
reason. The usual refusals:

- **Missing/invalid HMAC key** - CTS1 requires every uplink to be
  signed, so an absent or bad keyfile blocks *every* burst. The banner
  reads `(MISSING)` / `(BAD)`, and `--self-test` prints the exact path
  it looked for on the `hmac:` line. Put the key there, or pass
  `--hmac-keyfile <path>`.
- **`allow-tx` not ticked** in the compose modal (it resets each pass).
- **`--no-tx`** (preview only) or **`--tx-dry-run`** (composes but never
  keys) on the command line - both show in `--self-test`.
- An **RX-only SDR** (RTL-SDR): transmit is unavailable by design; the
  RX panel shows `(RX-only)`.

If the log says `sent>` yet there is genuinely no RF or TRX LED on a
B210 clone, that points at the clone's FPGA not driving the transmit
ATR/PA the way stock UHD expects - re-check the
[clone FPGA setup](#sdr-backends).

**RX shows only noise / nothing decodes**

- Confirm the antenna is on the port the software receives on - for a
  B210 that is the `RX2` antenna of channel 0 (a clone may silk-screen
  it differently; `sdr_probe` names the ports).
- Check the status panel's carrier/Doppler is near the satellite's
  downlink, and the predictions panel shows the pass is actually
  overhead.
- Tune RX gain with `:gain <dB>` while watching the IQ peak/RMS on the
  RX panel - too low buries the signal, too high clips it.
- Render the last N seconds with `:spectrum <sec>` to see whether the
  carrier is even present before chasing the decoder.

**SDR unplugged mid-pass / `Abort trap: 6`**

Pulling the SDR's USB cable (or losing its power) while receiving raises
a fatal fault inside the radio library that can't be caught and resumed
from within the process. `simple_sat_ops` no longer dies raw and leaves
a wrecked terminal: it restores the terminal directly, closes the
waterfall window, prints one line about what happened and where the log
is, and exits cleanly. (If two threads hit the fault at once, only the
first runs the cleanup; the second waits for it to finish, so the
message prints and the screen is restored reliably.) Ctrl-C and
termination signals also quit through this same clean shutdown. If a
device error ever surfaces as an ordinary error rather than an abort,
the receive worker parks, the RX panel shows
`SDR DISCONNECTED - RX stopped`, and the rest of the program keeps
running. Fully *surviving* an unplug (rather than exiting cleanly) would
need the SDR in a separate process; that is designed and parked, not yet
built.

**`audit-overflow dropped=N`**

The audit-log writer thread fell behind the producer. Almost always
means another process holds an exclusive flock on `runs.log` for an
extended period (10s+). Check for stuck CI or other tooling
appending to the same file. The overflow line is informational;
in-flight events were dropped, not the persisted log.

## A note on feel

Everything in this manual can be read in an afternoon. The part that
takes longer, and that no manual can hand you, is feel: the operator's
growing sense of when a pass is going well and when it is quietly going
wrong. You start to read the Doppler drift without thinking about it.
You notice when the frame counter stalls a beat too long. You learn
the particular hesitation the rotator makes when it is about to lose
the satellite over the back of a high pass, and you reach for the stop
key before the panel has caught up.

This comes from passes, not pages. Run them as a viewer until the
rhythm is familiar, then run them with the controls. Keep the
captures; a pass that decoded badly is the best teacher you have.
Before long the software fades into the background, which is exactly
what good tools are supposed to do, and you are simply talking to a
satellite.

## Glossary

| Term | Meaning |
|------|---------|
| **AOS / LOS** | Acquisition of Signal / Loss of Signal. Pass start and end times. |
| **AX100** | The frame format used on FrontierSat's UHF link: 32-bit ASM, Golay-coded length, CCSDS scrambling, Reed-Solomon FEC. |
| **B210** | Ettus USRP B210 software-defined radio. Two-channel, 70 MHz to 6 GHz; receives on `RX2` and transmits on `TX/RX`, with receive running continuously through a transmit burst. |
| **CAT** | Computer-Aided Transceiver. The serial control protocol on the FT-991A (see [Appendix A](#appendix-a-what-didnt-stick---the-ft-991a-and-ic-9700-radio-path)). |
| **CSP** | Cubesat Space Protocol. The frame inside the AX100 wrapper. |
| **CTS-SAT-1 / FrontierSat** | Calgary-to-Space's first satellite. Often called "FrontierSat" in this codebase. |
| **Doppler** | The frequency shift from line-of-sight velocity. Computed each tick from the SGP4 range-rate. |
| **Flip mode** | Tracking mode for high-elevation passes ($>75^\circ$ peak). The boom swings through zenith on the back hemisphere instead of slewing $180^\circ$ in azimuth at apex. |
| **HMAC** | Hash-based Message Authentication Code. Every uplink frame carries one; the satellite drops unsigned frames. |
| **Pass folder** | `/FrontierSat/Operations/<yyyymmdd>/<HHMMLT>/`. Holds every artifact from one pass. |
| **PCM shadow** | The FM-demod-side decoder that runs in parallel to the live IQ chain. Counts frames only; doesn't drive the DB. |
| **Pursuit** | The whole-pass antenna trajectory planner that aims slightly ahead of the satellite so the rotator's constant-rate slew tracks the curve. |
| **Rot2ProG** | The SPID rotator controller. Talks W-frame BCD-ish over USB-serial. |
| **SPID** | The rotator manufacturer. |
| **SGP4 / SDP4** | The standard NORAD orbit propagators. SDP4 is the deep-space variant. |
| **TC (telecommand)** | A single uplink command, e.g. `CTS1+fs_list_directory_json(/,0,20)@tsexec=1779961244000!`. |
| **TLE** | Two-Line Element set, the standard format for NORAD orbital elements. We use the 3-line variant (name plus two element lines). |
| **TX compose modal** | The `t`-key modal in `simple_sat_ops` for hand-composing an uplink burst. |
| **Viewer** | A read-only `simple_sat_ops` instance connected to a running operator via IPC. |

---

## Appendix A: What didn't stick - the FT-991A and IC-9700 radio path

Every project has a road not taken, and this is ours. Before the B210
became the live RF path, the plan was the obvious one: drive a real
amateur transceiver. The code still builds, the tools below still run,
and on a good day you can key a frame through one of these radios. We
do not operate this way anymore, and it is worth saying plainly why,
because the reasons are instructive rather than embarrassing.

We started with an Icom IC-9700 and a used SignaLink into the Linux
box. The IC-9700 is advertised as having a "satellite mode," and for
working FM voice repeaters through a bird that is exactly true. But
its bandwidth for 9600-baud MSK data simply was not there. We worked
through every configuration the radio exposes - every menu, every
audio routing, every filter setting - short of opening the case to
look for capacitors to swap, and never got a clean data path out of
it.

So we moved to the Yaesu FT-991A, which became the default backend in
the code (the IC-9700 stayed as a secondary one, `radio-type=icom-civ`).
The Yaesu did better: moderate success on receive, enough that you
could *hear* the satellite's beacon bursts come up out of the noise as
clear pulses of static. But we could almost never decode them - the
frames came through corrupted nearly every time. We tried the obvious
fixes, then the unobvious ones, and eventually found ourselves
searching the forums for *hidden* FT-991A configuration settings.
That is usually the sign that you are asking a piece of equipment to
do something it was never built to do.

And that is the real lesson of this appendix. These are excellent
radios for what they are designed for: lower data rates, comfortable
signal-to-noise, or simple FM voice through a satellite repeater. A
9600-baud MSK cubesat downlink at the edge of the link budget is none
of those things. The B210 succeeded where both transceivers were
marginal because it puts the whole signal chain - filtering, timing,
demodulation - under software we control, instead of behind a front
panel and a fixed IF.

The FT-991A's specific frustrations on the data path are worth
recording, because they are concrete and they recur on any voice
radio pressed into data service:

* **The USB CODEC is low-pass filtered.** Tones up to about 12 kHz
  pass cleanly, but the 9600 GFSK pre-emphasis above roughly 5 kHz is
  attenuated, which rounds off exactly the edges the modem cares about.
  The intended cure is to route audio through the rear DATA jack with a
  SignaLink instead of the radio's own USB audio. That works, and it is
  also one more box, one more cable, and one more thing to get wrong at
  3 a.m.
* **The radio rewrites your settings behind your back.** The FT-991A
  force-resets Menu 072 (DATA PORT SELECT) to USB every single time it
  enters DATA-FM, regardless of what you wrote over CAT a moment
  earlier. `radio_yaesu_cat.c` deliberately does not fight this; the
  menu has to be pinned on the front panel by hand. This is the radio
  quirk promised in the foreword, and it is the kind that no amount of
  software will fix for you.
* **Half-duplex by hand.** Keying TX, releasing it, and getting back to
  RX cleanly meant choreographing CAT, audio, and PTT against a radio
  that has its own ideas about timing.

The B210 dissolved all three at once, and brought one quirk of its own
(an analog TX-LO that leaks if the transmit chain idles hot, see
`src/hw/b210_rx_tx_core.h`) - but that one we can hold inside our own
code rather than behind a front panel.

The radio path is preserved, not deleted: it is the right tool for
bench bring-up of a conventional station, and if a future ground
station needs a hardware transceiver the scaffolding is here. What
follows is how to use it.

### `radio_ctl`

CAT-control verification tool for the FT-991A. Most-used subcommands:

```sh
radio_ctl --radio-device=/dev/ttyUSB1 --radio-serial-speed=38400 \
          --store-device --store-serial-speed identify

radio_ctl set-freq 145.900e6
radio_ctl set-mode FM
radio_ctl set-mod-input acc      # rear DATA jack (SignaLink), default
radio_ctl set-mod-input usb      # rear USB CODEC
radio_ctl set-power 10
radio_ctl ptt on --allow-tx
radio_ctl ptt off
radio_ctl power on
radio_ctl power off
radio_ctl uplink-prep            # tune, FM mode, MOD source, DATA mode
```

`--store-device` and `--store-serial-speed` persist the values to
`~/.local/share/simple_sat_ops/radio_device` and `radio_serial_speed`.
Subsequent unadorned invocations pick them up.

The IC-9700 backend (`radio_icom_civ.c`) speaks binary CI-V with
BCD-packed payloads instead of the FT-991A's ASCII CAT, but presents
the identical command set to `radio_ctl`. Select it with
`--radio-type=icom-civ`; the FT-991A is `--radio-type=yaesu-cat`
(default).

**Safety.** TX-capable subcommands honour the same three TX safety
gates the operator UI does (see [TX safety gates](#tx-safety-gates)).
`ptt on` does nothing without `--allow-tx`; `ptt off` is always
allowed.

### FT-991A front-panel checklist

A handful of menu items must be set once on the radio's front panel.
They persist across power cycles, and several of them cannot be driven
reliably over CAT (Menu 072 is the one the radio keeps resetting, as
above). The authoritative list lives in the project root `CLAUDE.md`;
in brief:

| Menu | Setting | Why |
|------|---------|-----|
| 031 CAT RATE | match `--radio-serial-speed` (38400 is comfortable) | CAT link baud. |
| 033 CAT RTS | `DISABLE` | The USB-serial driver can't toggle RTS reliably; with it enabled the radio drops every CAT command. |
| 070 DATA IN SELECT | `REAR` | Pinned over CAT, but set it here too. |
| 071 DATA PTT SELECT | `DAKY` | Gates the rear-DATA audio path open under CAT keying. |
| 072 DATA PORT SELECT | `DATA` | The radio force-resets this to USB on entry to DATA-FM; the software does not fight it, so you must pin it here. |
| 079 PKT MODE | `9600` | The uplink baud. |

### ALSA TX helpers: `tx_tone`, `tx_white_noise`, `tx_frame`

ALSA-based TX helpers from the legacy transceiver path. `tx_tone` keys
the radio and plays a sine (or stepped sweep) through the modulator,
which is how the USB-CODEC bandwidth limit above was first measured.
`tx_white_noise` is the same idea with full-band noise for combined
TX/RX bandshape characterisation. `tx_frame` is a single-shot CSP and
AX100 frame transmitter from the command line. All three honour the TX
safety gates and use `radio_ctl`'s persistent device and baud.

### Persistent files

| Path | What's there |
|------|---------------|
| `~/.local/share/simple_sat_ops/radio_device` | Persisted transceiver tty path. |
| `~/.local/share/simple_sat_ops/radio_serial_speed` | Persisted transceiver baud. |

---

## Appendix B: Trial and error - the git repository

This whole project was built by trial and error, and the git history
is the record of it: not a tidy after-the-fact narrative but the
actual logbook of what was tried, what broke, and what fixed it. That
makes the repository unusually worth reading - and unusually hard to
walk into cold, because the one thing it will not tell you on its own
is where to start. This appendix is that starting point.

**The headline: `main` is not the system you run.** `main` is the
published baseline. It is frozen at one reviewed merge,
`1dd038a` (2026-05-02, "multi-radio: pluggable backends, FT-991A
canonical, RX/TX tooling (#3)"), and it is deliberately a long way
behind. The live ground station, everything this manual describes, is
the **`iq-inspector`** branch. If you clone this repo and build
`main`, you will get the radio-first design from
[Appendix A](#appendix-a-what-didnt-stick---the-ft-991a-and-ic-9700-radio-path),
not the SDR system. Start with:

```sh
git checkout iq-inspector
```

### The active line

The branch you want grew commit by commit into one long, *linear*
chain: no merge commits and very little rebasing, just one small
change after another. Read end to end it is the development history of
the SDR ground station, in four eras. The ranges below are `git log`
arguments you can paste directly.

| Era (branch) | Commit range | Dates | What it introduced |
|--------------|--------------|-------|--------------------|
| `multi-operator` | `1dd038a..afe29d4` (~107) | 05-02 to 05-15 | The B210 becomes the live RF path. SDR RX/TX chain, the `rx_session` decode loop, the shared packet database and `packet_browser`, the SatNOGS audio pull, and the SatNOGS-style IQ waterfall. Absorbs the `SDR`, `packet-browser`, and `satnogs-audio-pull` topic branches. |
| `coherent-iq-demod` | `afe29d4..394b7c7` (~66) | 05-15 to 05-19 | A shadow IQ-domain demodulator running alongside the FM path for A/B sensitivity, coherent demod work, and a long run of format-truncation fixes. |
| `live-graphics` | `394b7c7..00f6ba6` (~67) | 05-19 to 05-24 | raylib graphics: the `live_waterfall` real-time spectrogram and the `iq_annotator` waveform panel (phase and split-channel views), plus `--tx-dry-run`. |
| `iq-inspector` (current) | `00f6ba6..0246372` (~66) | 05-24 to 05-28 | `decode_inspector` (the renamed `iq_annotator`): a staged decoder visualizer walking ASM, Golay, descrambler, Reed-Solomon, and CSP, with a Viterbi slicer and a `--live` mode. Then AD9361 tracking-loop defaults, the T/R switch driver (`14b8d23`), the B210 TX-LO-leak fix (`86c5c6d`, power the TX chain down between bursts), `agenda_check --tle`/`--prune-dups`, `next_in_queue --tle`, the async threading pass (TX, rotator, and audit log on worker threads), elevation jog keys, rotator slew-rate calibration with the pursuit planner, and the one-operator `--control` refusal. |

To walk any era:

```sh
git log --oneline 1dd038a..afe29d4          # the multi-operator era
git log -p --reverse 00f6ba6..0246372 -- apps/main.c   # the current era, one file
```

### How the commits are meant to be read

The commit history is not bookkeeping; it is the troubleshooting
record. Development happened across two machines. Why a Mac dev host,
you ask? No grand reason - it happened to be Burchill's work laptop,
and it had Claude Code installed, so that is where the code got
written. It builds the non-hardware tools (no ALSA, no SGP4SDP4, no
UHD by default); the actual radio, rotator, and antenna live on the
RAO ground station. So the loop was: write and commit on the Mac,
push, pull on RAO, test against the hardware, and watch for errors,
limitations, or features worth adding - then back to the Mac to do it
again. The commits are
therefore deliberately small, one observable change each, so that a
regression can be cornered with `git bisect` and a behavior can be
explained with `git log -p`. A run of forty near-identical
`decode_inspector:` commits is not noise; it is one debugging session,
written down. When you want to know *why* a line is the way it is, the
commit message that introduced it is usually the answer:

```sh
git log -p -S'the string or symbol you are chasing' iq-inspector
git bisect start iq-inspector 1dd038a       # good=baseline, find the change
```

`main` advances only by reviewed pull request, squash-merged, which is
why its tip is a single squashed commit (`1dd038a`, "(#3)") rather than
a record of the branch behind it, and why it lags. Treat it as the
stable tag you would hand someone, not as the working copy.

### The other branches

Everything else is either a topic branch that fed the active line, a
parallel experiment, or a fossil from before the SDR era. None of them
is where you should be working.

| Branch | Date | Status |
|--------|------|--------|
| `iq-inspector` | 2026-05-28 | **The live system.** Work here. |
| `main` | 2026-05-02 | Published baseline; one reviewed merge behind by design. |
| `tr-switch` | 2026-05-26 | Bench branch for the T/R antenna switch. The driver was reimplemented on the active line at `14b8d23`; this tip is the parallel take. |
| `live-spectrum` | 2026-05-11 | `rx_tui` ribbon and gridline experiments. Superseded by the operator UI panels. |
| `parsing` | 2026-05-04 | `rx_decode`/`rx_replay` parsing and `--help` docs. Superseded by `decode_inspector`. |
| `SDR`, `packet-browser`, `satnogs-audio-pull` | 2026-05 | Topic branches that were folded into the `multi-operator` era. Reachable from `iq-inspector`; kept for reference. |
| `tx-tone` | 2026-04-27 | The `--tx-power` control and high-power safety gate. Reached `main` via PR #3; this branch is the un-squashed history. Archived. |
| `telemetry` | 2026-04-19 | Early telemetry stubs, pre-SDR. Origin-only. |
| `rao` | 2025-02 | The original ground-station branch that ran on the RAO machine. The root of the Mac-versus-remote split. |
| `mac`, `usb-direct` | 2025-02 | The Mac dev-host build fixes and the early USB-serial radio experiments. Fossils. |

If you are new and reading this: check out `iq-inspector`, build it
per [Build and install](#build-and-install), and run a pass as a
viewer per the drill in
[Modes: operator vs. viewer](#modes-operator-vs-viewer). The rest of
the map is here for the day you need to know why something changed,
which on a system debugged one commit at a time is a day that comes
often.
